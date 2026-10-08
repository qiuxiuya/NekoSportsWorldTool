//! 全链编排：policy → 实时点位 → 轨迹生成 → 提交 → OBS → 详情验证。
//! 由 UI 后台线程调用，log 闭包回传日志。

use super::client::ApiClient;
use super::model::Session;
use super::points;
use super::policy::fetch_policy;
use super::records::fetch_one_record;
use super::submit::{submit_record, SubmitParams, SubmitResult};
use crate::location::Coordinate;
use crate::track::generate_road::RouteMode;
use crate::track::generator::build as gen_track;
use crate::track::wire::{build_obs_object, five_point_wrapper_with_area, obs_keys};
use rand_distr::{Distribution, Normal};
use serde_json::Value;

/// 自定义 / 高德路径（BD-09 折线）与所选建筑 SDF 数据。
///
/// `points_bd` 为用户给定顺序的坐标（已转 BD-09）；`buildings_bd` 可为空；
/// `close` 为首尾走法（循环 / 往返 / 单程）。
#[derive(Clone, Debug, Default)]
pub struct CustomRoute {
    pub points_bd: Vec<(f64, f64)>,
    pub buildings_bd: Vec<Vec<(f64, f64)>>,
    pub close: crate::track::generate_road::PathClose,
}

#[derive(Clone)]
pub struct RunParams {
    /// 距离（米）与时长（秒）已由 UI 参数解析。
    pub dist: f64,
    pub dur: i64,
    /// 开始时间（毫秒）。
    pub start_ms: i64,
    pub face_check: i64,
    /// 用户手动填写的绝对海拔（米）；None 使用生成器海拔。
    pub manual_altitude: Option<f64>,
    /// 用户手动填写的海拔范围；与单值字段兼容，范围优先。
    pub manual_altitude_range: Option<crate::track::altitude::AltitudeRange>,
    pub seed: u64,
    /// 路线算法模式。
    pub route_mode: RouteMode,
    /// 自定义路径（仅 `RouteMode::Custom` 使用）；None 或点不足时回退经典算法。
    pub custom_route: Option<CustomRoute>,
    /// GPS 漂移距离（米）：相关漂移稳态幅度。
    pub gps_drift_m: f64,
}

pub struct RunOutcome {
    pub result: SubmitResult,
    pub obs_ok: usize,
    pub detail_ok: bool,
}

fn sleep_secs(s: u64) {
    std::thread::sleep(std::time::Duration::from_secs(s));
}

/// 跑步全链。
pub fn run_full_flow(
    client: &mut ApiClient,
    params: &RunParams,
    log: &mut dyn FnMut(&str),
) -> Result<RunOutcome, String> {
    let sess: Session = client.login.clone().ok_or("未登录")?;

    // ① policy
    log("[policy] 拉取跑步策略…");
    let pol = fetch_policy(client)?;
    log(&format!(
        "√ [policy] ts={} policy={} minDistance={} validTime={}",
        pol.timestamp, pol.policy, pol.min_distance, pol.valid_time
    ));
    sleep_secs(2);

    // ② 实时点位（拒绝本地样本兜底）
    log("[points] 拉取实时点位…");
    if client.identity.has_unconfigured_default_location() {
        return Err("请先在设备信息页填写本次跑步所在城市和定位锚点，不能使用大连默认配置".into());
    }
    let anchor: Coordinate = client.identity.anchor_coordinate()?;
    // 携带策略下发的 runAreaId 拉点位；点位响应回传围栏（详情页绿色边界/目标点）
    let requested_area_id = (pol.area.run_area_id >= 0).then(|| pol.area.run_area_id.to_string());
    let mut points_ctx = points::fetch_points_context_ext(client, anchor, requested_area_id, log)?;
    // 策略下发的区域信息优先（与 App 一致）
    if pol.area.run_area_id >= 0 {
        points_ctx.area.run_area_id = pol.area.run_area_id;
    }
    if pol.area.geo_fences_json.trim() != "[]"
        && pol.area.freedom_show_fence
        && serde_json::from_str::<Value>(&pol.area.geo_fences_json).is_ok()
    {
        points_ctx.area.geo_fences_json = pol.area.geo_fences_json.clone();
        points_ctx.area.freedom_show_fence = pol.area.freedom_show_fence;
    }
    let pts = points_ctx.points.clone();
    // 学校明确无点位（10600）：允许继续，后续回退自由跑；完全不透明地拿到空列表才拒绝。
    if pts.is_empty() && !points_ctx.no_points {
        return Err("实时点位为空 —— 拒绝本地样本兜底".into());
    }
    log(&format!(
        "√ [points] {} 个点位，runAreaId={}，绿色围栏={}（{} 字节）{}",
        pts.len(),
        points_ctx.area.run_area_id,
        points_ctx.area.freedom_show_fence,
        points_ctx.area.geo_fences_json.len(),
        if points_ctx.no_points { "（学校未设置点位，回退自由跑）" } else { "" },
    ));
    for p in pts.iter().take(5) {
        log(&format!(
            "  [points] {} BD=({:.6},{:.6}) GCJ=({},{})",
            p.get("pointName").and_then(|v| v.as_str()).unwrap_or(""),
            p.get("lat").and_then(|v| v.as_f64()).unwrap_or(0.0),
            p.get("lon").and_then(|v| v.as_f64()).unwrap_or(0.0),
            p.get("glat").map(|v| v.to_string()).unwrap_or_default(),
            p.get("glon").map(|v| v.to_string()).unwrap_or_default(),
        ));
    }

    // ③ 轨迹生成（必经点 + 打卡点）
    let pts_bd = points::points_bd(&pts);
    // 必经点保持策略顺序置于前端（waypoints[0] 即起点），剩余打卡点去重后按质心角
    // 排序，使环序自然且不破坏必经点顺序。
    let mut route_pts: Vec<(f64, f64)> = pol.must_points.clone();
    let mut free: Vec<(f64, f64)> = Vec::new();
    for p in &pts_bd {
        if !route_pts
            .iter()
            .any(|q| (q.0 - p.0).abs() < 1e-6 && (q.1 - p.1).abs() < 1e-6)
        {
            free.push(*p);
        }
    }
    if !free.is_empty() {
        route_pts.extend(crate::track::generate_road::radial_order(&free));
    }
    if !pol.must_points.is_empty() {
        log(&format!(
            "√ [policy] 必经点 {} 个（保持顺序），合并后路线 waypoint 共 {} 个",
            pol.must_points.len(),
            route_pts.len()
        ));
        // 防御性日志：must_points 的「首个=起点」语义未经证实（提交时 skip(1) 依赖此假设），
        // 逐点打印便于抓真实响应核对，避免首个必经点被静默丢弃。
        for (i, &(mlat, mlon)) in pol.must_points.iter().enumerate() {
            log(&format!(
                "  [policy] must_points[{i}] BD=({mlat:.6},{mlon:.6}){}",
                if i == 0 { "（假设为起点）" } else { "（必经点）" }
            ));
        }
    } else {
        log(&format!(
            "[policy] 响应未含必经点列表，仅用打卡点 {} 个",
            route_pts.len()
        ));
    }
    if let Some(&(slat, slon)) = route_pts.first() {
        log(&format!("√ [track] 起点 BD=({slat:.6},{slon:.6})"));
    }
    // 平均配速须落在有效窗口内（否则逐点速度无法全窗内），越界时修正时长
    let mut params = params.clone();
    let avg = params.dist / params.dur as f64;
    let fixed_avg = avg.clamp(
        crate::track::generator::SPEED_FLOOR + 0.1,
        crate::track::generator::SPEED_CEIL - 0.1,
    );
    if (fixed_avg - avg).abs() > 1e-6 {
        let fixed_dur = (params.dist / fixed_avg).round() as i64;
        log(&format!(
            "[track] 平均配速 {} m/s 超出有效窗口，时长 {} -> {}s",
            (avg * 100.0).round() / 100.0,
            params.dur,
            fixed_dur
        ));
        params.dur = fixed_dur;
    }
    log(&format!(
        "[track] 生成轨迹 {:.0}m / {}s（{} 点位）…",
        params.dist,
        params.dur,
        route_pts.len()
    ));
    // 随机 0-4 秒偏移（终端上报的 flag 与首点差 <5s），轨迹/提交/OBS/五点统一使用
    let start_ms = params.start_ms + (rand::random::<i64>() % 5) * 1000;
    let mut track = match params.route_mode {
        RouteMode::Road => {
            let cfg = crate::api::model::load_config();
            if cfg.osm_path.is_empty() {
                log("⚠ [track] 未配置 OSM 路网，回退经典算法");
                gen_track(
                    params.dist,
                    params.dur,
                    params.seed,
                    (anchor.latitude, anchor.longitude),
                    start_ms,
                    &pts_bd,
                    params.gps_drift_m,
                )
            } else {
                match crate::track::generate_road::load_network_path(&cfg.osm_path) {
                    Ok(mut net) => {
                        crate::track::generate_road::align_network(&mut net);
                        // 电子围栏：裁剪到围栏内道路（失败/无围栏则跳过）
                        let fences = match crate::api::fence::fetch_geo_fence(client) {
                            Ok(f) => {
                                let _ = crate::api::model::save_fence_cache(&f);
                                log(&format!("√ [track] 电子围栏 {} 个", f.len()));
                                f
                            }
                            Err(e) => {
                                log(&format!("⚠ [track] 围栏获取失败，回退缓存: {e}"));
                                crate::api::model::load_fence_cache().unwrap_or_default()
                            }
                        };
                        let filtered = crate::track::generate_road::apply_fences(&net, &fences);
                        // 强制必经点（不含起点）：其余打卡点仅软引导 + <40m 吸附
                        let must_bd: Vec<(f64, f64)> =
                            pol.must_points.iter().skip(1).copied().collect();
                        match crate::track::generate_road::build_road(
                            params.dist,
                            params.dur,
                            params.seed,
                            start_ms,
                            &route_pts,
                            &must_bd,
                            &filtered,
                            params.gps_drift_m,
                        ) {
                            Ok(t) => {
                                log(&format!(
                                    "√ [track] 真实道路路由 {} 点 / {} 建筑 / {} 围栏",
                                    t.locations.len(),
                                    filtered.buildings.len(),
                                    fences.len()
                                ));
                                t
                            }
                            Err(e) => {
                                log(&format!("⚠ [track] 道路路由失败，回退经典算法: {e}"));
                                gen_track(
                                    params.dist,
                                    params.dur,
                                    params.seed,
                                    (anchor.latitude, anchor.longitude),
                                    start_ms,
                                    &pts_bd,
                                    params.gps_drift_m,
                                )
                            }
                        }
                    }
                    Err(e) => {
                        log(&format!("⚠ [track] 路网加载失败，回退经典算法: {e}"));
                        gen_track(
                            params.dist,
                            params.dur,
                            params.seed,
                            (anchor.latitude, anchor.longitude),
                            start_ms,
                            &pts_bd,
                            params.gps_drift_m,
                        )
                    }
                }
            }
        }
        // 自定义路径与高德路径几何源一致：均为按给定折线走。
        RouteMode::Custom | RouteMode::Amap => match params.custom_route.as_ref() {
            Some(cr) if cr.points_bd.len() >= 2 => {
                match crate::track::generate_road::build_custom(
                    params.dist,
                    params.dur,
                    params.seed,
                    start_ms,
                    &cr.points_bd,
                    cr.close,
                    &cr.buildings_bd,
                    params.gps_drift_m,
                ) {
                    Ok(t) => {
                        log(&format!(
                            "√ [track] 折线路径 {} 点 / {} 建筑 / {}",
                            t.locations.len(),
                            cr.buildings_bd.len(),
                            cr.close.label()
                        ));
                        t
                    }
                    Err(e) => {
                        log(&format!("⚠ [track] 折线路径生成失败，回退经典算法: {e}"));
                        gen_track(
                            params.dist,
                            params.dur,
                            params.seed,
                            (anchor.latitude, anchor.longitude),
                            start_ms,
                            &pts_bd,
                            params.gps_drift_m,
                        )
                    }
                }
            }
            _ => {
                log("⚠ [track] 折线路径未配置或有效点不足，回退经典算法");
                gen_track(
                    params.dist,
                    params.dur,
                    params.seed,
                    (anchor.latitude, anchor.longitude),
                    start_ms,
                    &pts_bd,
                    params.gps_drift_m,
                )
            }
        },
        RouteMode::Legacy => gen_track(
            params.dist,
            params.dur,
            params.seed,
            (anchor.latitude, anchor.longitude),
            start_ms,
            &pts_bd,
            params.gps_drift_m,
        ),
    };
    if let Some(range) = params.manual_altitude_range {
        crate::track::altitude::override_bd_a_range(&mut track, range)?;
        log(&format!(
            "√ [track] 已将海拔曲线映射到 {:.2}-{:.2}m，覆盖 {} 个点，爬升/圈数据将按覆盖值计算",
            range.min_m,
            range.max_m,
            track.locations.len()
        ));
    } else if let Some(altitude_m) = params.manual_altitude {
        crate::track::altitude::override_bd_a(&mut track, altitude_m)?;
        log(&format!(
            "√ [track] 已用手动海拔 {:.2}m 覆盖 {} 个点，爬升/圈数据将按覆盖值计算",
            altitude_m,
            track.locations.len()
        ));
    }

    // 城市自动获取：开启后按轨迹起点逆地理编码（有高德 Key 用高德，否则免费 OSM），失败即报错中断。
    if client.identity.city_auto {
        let start = track
            .locations
            .first()
            .ok_or("轨迹为空，无法自动获取城市")?;
        log(&format!(
            "[city] 城市自动模式：按轨迹起点({:.6},{:.6}) 逆地理编码…",
            start.gLat, start.gLng
        ));
        let city = crate::api::amap::regeo_city_osm_bd(start.gLat, start.gLng)
            .map_err(|e| format!("城市自动获取失败：{e}；可关闭城市自动或检查网络"))?;
        log(&format!("√ [city] 已自动获取城市「{city}」"));
        client.identity.city = city;
        if let Err(e) = super::model::save_identity(&client.identity) {
            log(&format!("⚠ 城市持久化失败: {e}"));
        }
    }
    log(&format!(
        "√ [track] {} 点 totalDis={:.0}m steps={} 起点={}",
        track.locations.len(),
        track.totalDistance,
        track.totalSteps,
        chrono::Local
            .timestamp_millis_opt(params.start_ms)
            .single()
            .map(|t| t.format("%H:%M:%S").to_string())
            .unwrap_or_default(),
    ));

    // 锚点更新并持久化：下次拉点位即真实坐标，摆脱写死的默认值。
    // 自动模式：以轨迹起点为基准，按设定距离/方位角偏移（设备页可配置）；
    // 关闭自动：沿用打卡点随机漂移，保持原有行为。
    {
        let new_anchor_bd: Option<(f64, f64)> = match track.locations.first() {
            Some(start) => match client
                .identity
                .auto_anchor(start.gLat, start.gLng)
            {
                Some((lat, lon)) => {
                    log(&format!(
                        "[points] 自动锚点：起点({:.6},{:.6}) 按 {:.0}m / {:.0}° 偏移 → ({lat:.6},{lon:.6})",
                        start.gLat,
                        start.gLng,
                        client.identity.anchor_offset_m,
                        client.identity.anchor_offset_bearing,
                    ));
                    Some((lat, lon))
                }
                None => {
                    if pts_bd.is_empty() {
                        None
                    } else {
                        let idx = (rand::random::<f64>() * pts_bd.len() as f64) as usize;
                        let (clat, clng) = pts_bd[idx];
                        let mut rng = rand::thread_rng();
                        let normal = Normal::<f64>::new(0.0, 120.0).unwrap();
                        let dlat = normal.sample(&mut rng).clamp(-200.0, 200.0)
                            / crate::track::geom::MET_PER_DEG_LAT;
                        let dlng = normal.sample(&mut rng).clamp(-200.0, 200.0)
                            / crate::track::geom::MET_PER_DEG_LNG;
                        Some((clat + dlat, clng + dlng))
                    }
                }
            },
            None => None,
        };
        if let Some((lat, lon)) = new_anchor_bd {
            client.identity.anchor_lat = lat;
            client.identity.anchor_lon = lon;
            if let Err(e) = super::model::save_identity(&client.identity) {
                log(&format!("⚠ 锚点持久化失败: {e}"));
            }
            // 用新锚点重存点位缓存，使缓存锚点与持久化锚点一致，避免预览锚点失配；
            // 同时保留区域元数据，供下次详情页绿色围栏使用。
            if let Ok(new_anchor) = client.identity.anchor_coordinate() {
                let _ = super::model::save_points_cache_context(new_anchor, &pts, &points_ctx.area);
            }
        }
    }
    let (ascent, descent, net) = track.elevation_stats();
    log(&format!(
        "[track] 海拔统计：起点 {:.2}m，终点 {:.2}m，累计爬升 {:.2}m，累计下降 {:.2}m，净变化 {:.2}m",
        track.locations.first().map(|point| point.bdA).unwrap_or(0.0),
        track.locations.last().map(|point| point.bdA).unwrap_or(0.0),
        ascent,
        descent,
        net,
    ));

    // 无点位学校（10600）：回退自由跑——不传五点、policy 置 0、跳过五点校验。
    // 有打卡点：五点 wrapper（跑完态，record body 与 OBS fixed_point_json 共用），
    // 携带服务端区域元数据（runAreaId + geoFencesJson），否则详情页只显示灰线、无目标点。
    let free_run = points_ctx.no_points || pts.is_empty();
    let (five, five_wrap) = if free_run {
        log("⚠ [points] 学校未设置点位，按自由跑提交（不传 fivePointJson）");
        (String::new(), String::new())
    } else {
        let w = five_point_wrapper_with_area(&pts, track.startTime, &points_ctx.area);
        (w.clone(), w)
    };
    let policy_value = if free_run { 0 } else { pol.policy };

    // ⑤ 提交（sportType=1）
    log("[record] 提交跑步记录（sportType=1）…");
    let sp = SubmitParams {
        track,
        uid: sess.uid,
        selected_unid: sess.unid.parse().unwrap_or(0),
        policy: policy_value,
        policy_ts: pol.timestamp,
        min_distance: pol.min_distance,
        weight: if sess.weight > 0.0 { sess.weight } else { 68.0 },
        face_check: params.face_check,
        five_point_json: five,
        address: client.identity.city.clone(),
    };
    let result = submit_record(client, &sp, log)?;
    sleep_secs(1);

    // ⑥ OBS 上传（双 key）
    // 从提交结果回填 track.startTime（含随机秒偏移），保证 body/OBS/flag 全链一致
    let mut track_for_obs = sp.track.clone();
    track_for_obs.startTime = result.start_ms;
    // OBS fixed_point_json wrapper 同步携带 rrid 后的窗序（与 record 通道数组串共存）
    // 自由跑无五点：传 None（按 live_points 现拼；pts 空则 fixed 为空）
    let five_opt = (!five_wrap.is_empty()).then_some(five_wrap.as_str());
    let obj = build_obs_object(&track_for_obs, result.rrid, &result.uuid, sess.uid, &pts, five_opt);
    let seg_n = crate::track::wire::segment_count(&track_for_obs);
    log(&format!(
        "[obs] 上传 OBS 对象（gzip+base64，11 键；分段 {seg_n} 段 state=0；runAreaId={}）…",
        points_ctx.area.run_area_id
    ));
    let payload = obj.to_string().into_bytes();
    let keys = obs_keys(&track_for_obs, result.rrid, &result.uuid);
    let obs_ok = super::obs::upload_both_keys(client, &keys, &payload, log);
    if obs_ok == 2 {
        log("√ [obs] 双 key 上传成功");
    } else {
        log(&format!("⚠ [obs] 上传成功 {obs_ok}/2"));
    }

    // ⑦ 详情验证
    sleep_secs(2);
    log("[verify] 拉取详情验证…");
    let detail_ok = match fetch_one_record(client, result.rrid) {
        Ok(d) => {
            log(&format!(
                "√ [verify] rrid={} complete={:?} dis={:?} time={:?}",
                result.rrid,
                d.get("complete").and_then(|v| v.as_bool()),
                d.get("totalDis"),
                d.get("totalTime"),
            ));
            if let Ok(mut slot) = VERIFY_DETAIL.lock() {
                slot.replace(d.clone());
            }
            true
        }
        Err(e) => {
            log(&format!(
                "⚠ [verify] 详情拉取失败（提交已成功 rrid={}）：{e}",
                result.rrid
            ));
            false
        }
    };
    Ok(RunOutcome {
        result,
        obs_ok,
        detail_ok,
    })
}

/// AI 提交流（UI 线程用）。
pub fn run_ai_submit(
    client: &mut ApiClient,
    sport_id: i64,
    mode: super::ai::AiMode,
    log: &mut dyn FnMut(&str),
) -> Result<Value, String> {
    log(&format!("[ai] 提交 sportId={sport_id} mode={mode:?}…"));
    let biz = super::ai::upload(client, sport_id, mode, None)?;
    log("√ [ai] 提交成功");
    Ok(biz)
}

/// AI 列表（UI 线程用）。
pub fn run_ai_list(
    client: &mut ApiClient,
    log: &mut dyn FnMut(&str),
) -> Result<Vec<super::ai::AiSport>, String> {
    log("[ai] 拉取项目列表…");
    let list = super::ai::fetch_list(client)?;
    log(&format!("√ [ai] {} 个项目", list.len()));
    Ok(list)
}

/// 记录列表（UI 线程用）。
pub fn run_records(
    client: &mut ApiClient,
    log: &mut dyn FnMut(&str),
) -> Result<Vec<super::records::RecordRow>, String> {
    log("[records] 拉取跑步记录…");
    let rows = super::records::fetch_records(client)?;
    log(&format!("√ [records] {} 条记录", rows.len()));
    Ok(rows)
}

use chrono::TimeZone as _;

/// 最近一次详情验证的完整响应（达标判定明细在 reasonList）。
pub static VERIFY_DETAIL: std::sync::Mutex<Option<Value>> = std::sync::Mutex::new(None);
