//! `runjob` 子命令：从一次 JSON 导入全部配置，自动登录 → 跑完全链 → 退出。
//!
//! 目标是一次性、可脚本化的「批量/无人值守」跑步：账号密码 + 设备身份 + 定位锚点 +
//! 跑步参数（距离/配速/开始时间/海拔/GPS 漂移/路线算法/自定义或高德路径等）全部由
//! 一个 JSON 文件提供，命令行只给文件路径即可。
//!
//! 用法：
//!   nekosportsworldtool runjob --file job.json
//!
//! JSON 结构（字段均可选，缺省沿用本地已保存配置；`username`/`password` 必需）：
//! ```json
//! {
//!   "username": "13800000000",
//!   "password": "secret",
//!   "dist_km_min": 5.0, "dist_km_max": 6.0,
//!   "pace_min": 360, "pace_max": 480,
//!   "start": { "mode": "random", "days_ago": 0 },
//!   "face_check": true,
//!   "altitude_min": 15.0, "altitude_max": 25.0,
//!   "gps_drift_m": 1.5,
//!   "route_mode": "road",
//!   "custom_close": "closed",
//!   "custom_datum": "bd09",
//!   "custom_points": [[38.901678, 121.540241], [38.902564, 121.541233]],
//!   "custom_use_buildings": true,
//!   "amap_key": "",
//!   "amap_jscode": "",
//!   "seed": 0,
//!   "device": { "platform": "android", "device_name": "22081212C", "os_version": "14" },
//!   "location": { "city": "北京市", "anchor_lat": 39.9042, "anchor_lon": 116.4074,
//!                 "auto": true, "offset_m": 200, "offset_bearing": 0, "city_auto": true }
//! }
//! ```
//! `start` 亦可写作 `{ "mode": "specified", "days_ago": 0, "time": "12:30" }`
//!（指定时刻）或 `{ "mode": "ago", "ago_min": 45 }`（45 分钟前）。
//! 距离/配速/海拔既可用固定值（`dist_km` / `pace` / `altitude`），也可用范围
//!（`dist_km_min`~`dist_km_max` / `pace_min`~`pace_max` / `altitude_min`~`altitude_max`）；
//! 路径三选一（优先级从高到低）：`custom_points`（内联数组 `[[纬度,经度],...]`）、
//! `custom_file`（外部文件路径）、`custom_text`（内联文本，`\n` 分行）。

use super::{fmt_hms, get, logger, now_ms, parse_flags, parse_pace};
use crate::api::client::ApiClient;
use crate::api::model::{self, HeaderIdentity};
use crate::track::generate_road::{PathClose, RouteMode};
use serde::Deserialize;

/// 开始时间配置。
#[derive(Debug, Clone, Deserialize)]
pub struct StartSpec {
    /// "random"（当日 7:00-20:00 内随机，同界面「随机时刻」）、
    /// "specified"（指定日期时刻）或 "ago"（多久以前，默认）。
    #[serde(default = "default_start_mode")]
    pub mode: String,
    /// ago 模式：分钟数。
    #[serde(default)]
    pub ago_min: i64,
    /// random / specified 模式：0=今天，1-3=前几天。
    #[serde(default)]
    pub days_ago: i64,
    /// specified 模式："HH:MM"。
    #[serde(default)]
    pub time: String,
}

fn default_start_mode() -> String {
    "ago".into()
}

/// 设备身份覆盖（未给出的字段继承本地 identity.json）。
#[derive(Debug, Clone, Default, Deserialize)]
pub struct DeviceSpec {
    #[serde(default)]
    pub platform: Option<String>,
    #[serde(default)]
    pub device_name: Option<String>,
    #[serde(default)]
    pub os_version: Option<String>,
    #[serde(default)]
    pub idfa: Option<String>,
    #[serde(default)]
    pub manufacturer: Option<String>,
    #[serde(default)]
    pub device_id: Option<String>,
}

/// 位置信息覆盖。
#[derive(Debug, Clone, Default, Deserialize)]
pub struct LocationSpec {
    #[serde(default)]
    pub city: Option<String>,
    #[serde(default)]
    pub anchor_lat: Option<f64>,
    #[serde(default)]
    pub anchor_lon: Option<f64>,
    /// 自动锚点：每次跑完以轨迹起点 + 距离/方位角偏移并保存。
    #[serde(default)]
    pub auto: Option<bool>,
    #[serde(default)]
    pub offset_m: Option<f64>,
    #[serde(default)]
    pub offset_bearing: Option<f64>,
    /// 城市自动：每次跑按轨迹起点逆地理编码。
    #[serde(default)]
    pub city_auto: Option<bool>,
}

/// runjob 的完整配置。
#[derive(Debug, Clone, Deserialize)]
pub struct RunJob {
    #[serde(default)]
    pub username: String,
    #[serde(default)]
    pub password: String,

    /// 固定距离（公里）；与 `dist_km_min`/`dist_km_max` 互斥，本字段优先。
    #[serde(default)]
    pub dist_km: Option<f32>,
    /// 距离范围下限（公里），与 `dist_km_max` 组成区间后随机抽样（同界面「距离范围」）。
    #[serde(default)]
    pub dist_km_min: Option<f32>,
    /// 距离范围上限（公里）。
    #[serde(default)]
    pub dist_km_max: Option<f32>,
    /// 固定配速；`"6:30"` 或秒数。与 `pace_min`/`pace_max` 互斥，本字段优先。
    #[serde(default)]
    pub pace: Option<String>,
    /// 配速范围下限（秒/km），与 `pace_max` 组成区间后随机（同界面「配速范围」）。
    #[serde(default)]
    pub pace_min: Option<f32>,
    /// 配速范围上限（秒/km）。
    #[serde(default)]
    pub pace_max: Option<f32>,
    #[serde(default)]
    pub start: Option<StartSpec>,
    #[serde(default)]
    pub face_check: Option<bool>,
    /// 固定海拔（米）；与 `altitude_min`/`altitude_max` 互斥，本字段优先。
    #[serde(default)]
    pub altitude: Option<f64>,
    /// 海拔范围下限（米），与 `altitude_max` 组成区间后映射（同界面「手动海拔」两框）。
    #[serde(default)]
    pub altitude_min: Option<f64>,
    /// 海拔范围上限（米）；与下限相同视为固定海拔。
    #[serde(default)]
    pub altitude_max: Option<f64>,
    #[serde(default)]
    pub gps_drift_m: Option<f32>,
    #[serde(default)]
    pub route_mode: Option<String>,
    #[serde(default)]
    pub custom_close: Option<String>,
    #[serde(default)]
    pub custom_datum: Option<String>,
    #[serde(default)]
    pub custom_text: Option<String>,
    /// 直接内联的路径点 `[[纬度, 经度], ...]`（与 `custom_text` 同基准），
    /// 优先级最高，免去外部文件与转义换行。
    #[serde(default)]
    pub custom_points: Option<Vec<Vec<f64>>>,
    /// 路径文本文件路径（GPX/GeoJSON/文本）；优先于 `custom_text`，避免在 JSON 里塞长文本。
    #[serde(default)]
    pub custom_file: Option<String>,
    #[serde(default)]
    pub custom_use_buildings: Option<bool>,
    #[serde(default)]
    pub amap_key: Option<String>,
    #[serde(default)]
    pub amap_jscode: Option<String>,
    #[serde(default)]
    pub seed: Option<u64>,

    #[serde(default)]
    pub device: Option<DeviceSpec>,
    #[serde(default)]
    pub location: Option<LocationSpec>,
}

/// `runjob --file <json>` 入口。
pub fn cmd_runjob(rest: &[&str]) -> i32 {
    let flags = parse_flags(rest);
    let path = match get(&flags, "file") {
        Some(p) => p,
        None => {
            eprintln!("缺少 --file <job.json>");
            return 2;
        }
    };
    let text = match std::fs::read_to_string(path) {
        Ok(t) => t,
        Err(e) => {
            eprintln!("读取 {path} 失败: {e}");
            return 2;
        }
    };
    let job: RunJob = match serde_json::from_str(&text) {
        Ok(j) => j,
        Err(e) => {
            eprintln!("解析 JSON 失败: {e}");
            return 2;
        }
    };
    match run(job) {
        Ok(()) => 0,
        Err(e) => {
            eprintln!("runjob 失败: {e}");
            1
        }
    }
}

fn run(job: RunJob) -> Result<(), String> {
    if job.username.trim().is_empty() || job.password.is_empty() {
        return Err("JSON 缺少 username / password".into());
    }
    let mut cfg = model::load_config();
    let mut identity = model::load_identity();

    apply_device(&mut identity, job.device.as_ref());
    apply_location(&mut identity, job.location.as_ref());
    apply_run_config(&mut cfg, &job);

    // 锚点未配置（大连默认值）时，优先用 JSON 里显式给出的锚点；仍缺则报错。
    if identity.has_unconfigured_default_location() {
        if let Some(loc) = job.location.as_ref() {
            if let (Some(lat), Some(lon)) = (loc.anchor_lat, loc.anchor_lon) {
                identity.anchor_lat = lat;
                identity.anchor_lon = lon;
            }
        }
    }
    if identity.anchor_coordinate().is_err() {
        return Err("定位锚点无效，请在 JSON 的 location.anchor_lat / anchor_lon 提供有效坐标".into());
    }

    model::save_identity(&identity)?;
    model::save_config(&cfg)?;

    // 登录：直接使用 JSON 凭据，不走 auto_login，避免沿用其它本地会话。
    let mut client = ApiClient::new(identity.clone(), None);
    let mut log = logger();
    let session = crate::api::login::login(&mut client, &job.username, &job.password, &mut log)?;
    println!(
        "登录成功 uid={} unid={} name={}",
        session.uid, session.unid, session.name
    );
    client.login = Some(session);

    // 路径参数（自定义 / 高德）。
    let route_mode = RouteMode::from_str(
        job.route_mode
            .as_deref()
            .unwrap_or(cfg.route_mode.as_str()),
    );
    let close = PathClose::from_str(
        job.custom_close
            .as_deref()
            .unwrap_or(cfg.custom_close.as_str()),
    );
    // 路径文本（仅折线类模式需要）：custom_points / custom_file / custom_text 三选一，
    // 优先级从高到低；均未提供则回退本地已存 custom_route.txt。
    let custom_inline = if route_mode.is_polyline_based() {
        inline_route_text(&job)?
    } else {
        String::new()
    };
    if !custom_inline.trim().is_empty() {
        model::save_custom_route(&custom_inline)?;
    }
    let custom_route = build_custom_route(&job, &cfg, route_mode, close, &custom_inline)?;

    // 运动量：距离（固定值 / 范围随机 / 缺省沿用本地 GUI 的距离范围）。
    let dist_km = match job.dist_km.filter(|d| *d > 0.0) {
        Some(v) => v,
        None => match (job.dist_km_min, job.dist_km_max) {
            (Some(a), Some(b)) => rand_between(a, b),
            (Some(a), None) => a,
            (None, Some(b)) => b,
            // 缺省沿用本地已保存的距离范围（同界面「距离范围」）。
            (None, None) => rand_between(cfg.dist_min, cfg.dist_max),
        },
    };
    let dist = dist_km as f64 * 1000.0;

    // 配速（固定值 / 区间随机 / 缺省沿用本地 GUI 的配速范围，秒/km）。
    let pace_s = match job
        .pace
        .as_deref()
        .filter(|p| !p.trim().is_empty())
        .map(parse_pace)
        .filter(|p| *p > 0.0)
    {
        Some(v) => v,
        None => match (job.pace_min, job.pace_max) {
            (Some(a), Some(b)) => rand_between(a, b),
            (Some(a), None) => a,
            (None, Some(b)) => b,
            // 缺省沿用本地已保存的配速范围（同界面「配速范围」）。
            (None, None) => rand_between(cfg.pace_min, cfg.pace_max),
        },
    };
    let dur = (dist as f32 / 1000.0 * pace_s) as i64;

    // 开始时间；`start` 缺省或 mode 未识别时，按当日随机时刻（同界面默认「随机时刻」）。
    let start_ms = match job.start.as_ref() {
        Some(s) if s.mode.eq_ignore_ascii_case("specified") => {
            specified_time(s.days_ago, &s.time)
        }
        Some(s) if s.mode.eq_ignore_ascii_case("random") => {
            random_in_window(s.days_ago, start_ms_latest(dur))
        }
        Some(s) if s.ago_min > 0 => now_ms() - s.ago_min * 60_000,
        _ => random_in_window(0, start_ms_latest(dur)),
    };

    // 海拔：固定值（altitude）优先，其次 altitude_min/altitude_max，缺省沿用本地已保存海拔。
    let (manual_altitude, manual_altitude_range) = resolve_altitude(&job, &cfg)?;

    let seed = match job.seed {
        Some(s) if s > 0 => s,
        _ => (now_ms() % 2_147_483_647) as u64,
    };
    let face_check = job.face_check.unwrap_or(cfg.face_check);
    let gps_drift_m = job.gps_drift_m.unwrap_or(cfg.gps_drift_m) as f64;

    println!(
        "参数：{:.0}m / {}s / 配速 {}:{:02}/km / 开始 {} / 路线 {}",
        dist,
        dur,
        pace_s as i64 / 60,
        pace_s as i64 % 60,
        fmt_hms(start_ms),
        route_mode.as_str()
    );

    let params = crate::api::flow::RunParams {
        dist,
        dur,
        start_ms,
        face_check: if face_check { 1 } else { 0 },
        manual_altitude,
        manual_altitude_range,
        seed,
        route_mode,
        custom_route,
        gps_drift_m,
    };
    let outcome = crate::api::flow::run_full_flow(&mut client, &params, &mut log)?;
    println!(
        "跑步提交成功 rrid={} uuid={} obs={}/2 verify={}",
        outcome.result.rrid,
        outcome.result.uuid,
        outcome.obs_ok,
        if outcome.detail_ok { "通过" } else { "未通过" }
    );
    Ok(())
}

fn apply_device(identity: &mut HeaderIdentity, spec: Option<&DeviceSpec>) {
    let Some(spec) = spec else { return };
    if let Some(v) = &spec.platform {
        identity.platform = v.clone();
    }
    if let Some(v) = &spec.device_name {
        identity.device_name = v.clone();
    }
    if let Some(v) = &spec.os_version {
        identity.os_version = v.clone();
    }
    if let Some(v) = &spec.idfa {
        identity.idfa = v.clone();
    }
    if let Some(v) = &spec.manufacturer {
        identity.manufacturer = v.clone();
    }
    if let Some(v) = &spec.device_id {
        identity.device_id = v.clone();
    }
}

fn apply_location(identity: &mut HeaderIdentity, spec: Option<&LocationSpec>) {
    let Some(spec) = spec else { return };
    if let Some(v) = &spec.city {
        identity.city = v.clone();
    }
    if let Some(v) = spec.anchor_lat {
        identity.anchor_lat = v;
    }
    if let Some(v) = spec.anchor_lon {
        identity.anchor_lon = v;
    }
    if let Some(v) = spec.auto {
        identity.anchor_auto = v;
    }
    if let Some(v) = spec.offset_m {
        identity.anchor_offset_m = v;
    }
    if let Some(v) = spec.offset_bearing {
        identity.anchor_offset_bearing = v;
    }
    if let Some(v) = spec.city_auto {
        identity.city_auto = v;
    }
}

fn apply_run_config(cfg: &mut model::Config, job: &RunJob) {
    if let Some(v) = job.route_mode.as_deref() {
        cfg.route_mode = RouteMode::from_str(v).as_str().into();
    }
    if let Some(v) = job.custom_close.as_deref() {
        cfg.custom_close = PathClose::from_str(v).as_str().into();
    }
    if let Some(v) = job.custom_datum.as_deref() {
        cfg.custom_datum = crate::track::custom::Datum::from_str(v).as_str().into();
    }
    if let Some(v) = job.custom_use_buildings {
        cfg.custom_use_buildings = v;
    }
    if let Some(v) = &job.amap_key {
        cfg.amap_key = v.clone();
    }
    if let Some(v) = &job.amap_jscode {
        cfg.amap_security_js_code = v.clone();
    }
    if let Some(v) = job.gps_drift_m {
        cfg.gps_drift_m = v;
    }
    if let Some(v) = &job.face_check {
        cfg.face_check = *v;
    }
    // 记下凭据，便于后续无参数 run 也能自动重登（与 GUI「记住密码」一致）。
    cfg.username = job.username.clone();
    cfg.password = job.password.clone();
    cfg.remember = true;
}

/// 构建自定义 / 高德路径（仅折线类路线模式需要）。
/// 解析路径文本来源：`custom_points`（内联数组）> `custom_file`（外部文件）> `custom_text`（内联）。
fn inline_route_text(job: &RunJob) -> Result<String, String> {
    if let Some(points) = job.custom_points.as_ref().filter(|p| !p.is_empty()) {
        let mut lines = Vec::with_capacity(points.len());
        for (i, p) in points.iter().enumerate() {
            if p.len() < 2 {
                return Err(format!("custom_points[{i}] 需要 [纬度, 经度] 两个数值"));
            }
            lines.push(format!("{},{}", p[0], p[1]));
        }
        return Ok(lines.join("\n"));
    }
    if let Some(path) = job.custom_file.as_deref().filter(|p| !p.trim().is_empty()) {
        return std::fs::read_to_string(path)
            .map_err(|e| format!("读取 custom_file {path} 失败: {e}"));
    }
    Ok(job.custom_text.clone().unwrap_or_default())
}

fn build_custom_route(
    job: &RunJob,
    cfg: &model::Config,
    route_mode: RouteMode,
    close: PathClose,
    inline: &str,
) -> Result<Option<crate::api::flow::CustomRoute>, String> {
    if !route_mode.is_polyline_based() {
        return Ok(None);
    }
    let text = if inline.trim().is_empty() {
        model::load_custom_route()
    } else {
        inline.to_string()
    };
    if text.trim().is_empty() {
        return Err(
            "路线为自定义/高德模式，但未提供 custom_points / custom_file / custom_text 且本地无已保存路径"
                .into(),
        );
    }
    let datum = crate::track::custom::Datum::from_str(
        job.custom_datum.as_deref().unwrap_or(cfg.custom_datum.as_str()),
    );
    let parsed = crate::track::custom::parse_route(&text, datum)?;
    let points_bd = if route_mode == RouteMode::Amap {
        let amap_cfg = crate::api::amap::AmapConfig {
            key: job
                .amap_key
                .clone()
                .unwrap_or_else(|| cfg.amap_key.clone()),
            jscode: job
                .amap_jscode
                .clone()
                .unwrap_or_else(|| cfg.amap_security_js_code.clone()),
        };
        if !amap_cfg.is_ready() {
            return Err("高德模式需提供 amap_key".into());
        }
        let mut seq_gcj: Vec<(f64, f64)> = parsed
            .points_bd
            .iter()
            .map(|&(la, lo)| crate::track::wire::bd09_to_gcj02(la, lo))
            .collect();
        match close {
            PathClose::Closed => {
                let first = seq_gcj[0];
                let last = *seq_gcj.last().unwrap();
                if (first.0 - last.0).abs() > 1e-9 || (first.1 - last.1).abs() > 1e-9 {
                    seq_gcj.push(first);
                }
            }
            PathClose::RoundTrip => {
                let mut back: Vec<(f64, f64)> =
                    seq_gcj[..seq_gcj.len() - 1].iter().rev().copied().collect();
                seq_gcj.append(&mut back);
            }
            PathClose::OneWay => {}
        }
        let mut lg = |s: &str| println!("{}", crate::textlog::clean(s));
        let route = crate::api::amap::plan_walking(&amap_cfg, &seq_gcj, &mut lg)?;
        println!(
            "高德步行路径：{} 点，约 {:.0} m",
            route.points_bd.len(),
            route.length_m
        );
        route.points_bd
    } else {
        println!("自定义路径：{} · {} 个点", parsed.format, parsed.points_bd.len());
        parsed.points_bd
    };
    let use_buildings = job.custom_use_buildings.unwrap_or(cfg.custom_use_buildings);
    let buildings_bd = if use_buildings {
        crate::track::generate_road::load_buildings_bd(&cfg.osm_path)
    } else {
        Vec::new()
    };
    Ok(Some(crate::api::flow::CustomRoute {
        points_bd,
        buildings_bd,
        close,
    }))
}

/// 解析海拔配置：`altitude`（固定值，优先）或 `altitude_min`/`altitude_max` 区间。
///
/// 复用 [`parse_fields`](crate::track::altitude::parse_fields)，与界面「手动海拔」两框语义一致：
/// 两值相同视为固定海拔；两者都为空则返回 (None, None)（用生成器海拔曲线）。
fn resolve_altitude(
    job: &RunJob,
    cfg: &model::Config,
) -> Result<(Option<f64>, Option<crate::track::altitude::AltitudeRange>), String> {
    if let Some(v) = job.altitude {
        return Ok((Some(v), None));
    }
    match (job.altitude_min, job.altitude_max) {
        // 缺省沿用本地已保存海拔（固定值或范围；都没有则用生成器海拔曲线）。
        (None, None) => Ok((cfg.manual_altitude, cfg.manual_altitude_range)),
        // 两值相同 = 固定海拔（与界面「两框相同=固定海拔」一致，须在 parse_fields 前判定）。
        (Some(a), Some(b)) if (a - b).abs() < 1e-9 => Ok((Some(a), None)),
        (min, max) => parse_altitude_bounds(min, max),
    }
}

/// 把 `altitude_min`/`altitude_max`（可能只填一侧）转成海拔规格；失败返回 `Err`。
fn parse_altitude_bounds(
    min: Option<f64>,
    max: Option<f64>,
) -> Result<(Option<f64>, Option<crate::track::altitude::AltitudeRange>), String> {
    let min_text = min.map(|v| v.to_string()).unwrap_or_default();
    let max_text = max.map(|v| v.to_string()).unwrap_or_default();
    match crate::track::altitude::parse_fields(&min_text, &max_text)? {
        None => Ok((None, None)),
        Some(crate::track::altitude::AltitudeSpec::Single(x)) => Ok((Some(x), None)),
        Some(crate::track::altitude::AltitudeSpec::Range(r)) => Ok((None, Some(r))),
    }
}

/// 在 [a, b] 内均匀随机（自动纠正顺序）。
fn rand_between(a: f32, b: f32) -> f32 {
    let (lo, hi) = if a <= b { (a, b) } else { (b, a) };
    lo + rand::random::<f32>() * (hi - lo)
}

/// 开始时间的「不晚于此」上界：当前时刻 - 用时 - 抖动余量，避免生成「未来开跑」。
fn start_ms_latest(dur: i64) -> i64 {
    now_ms() - dur * 1000 - 5_000
}

/// 指定日期（days_ago 天前）的 7:00-20:00 内随机抽样，且不晚于 `latest_ms`。
///
/// 与界面「随机时刻」一致：窗口整体晚于 `latest_ms` 时退化为 `latest_ms`。
fn random_in_window(days_ago: i64, latest_ms: i64) -> i64 {
    use chrono::{Datelike, TimeZone};
    let base = chrono::Local::now() - chrono::Duration::days(days_ago.clamp(0, 3));
    let lo = chrono::Local
        .with_ymd_and_hms(base.year(), base.month(), base.day(), 7, 0, 0)
        .single();
    let hi = chrono::Local
        .with_ymd_and_hms(base.year(), base.month(), base.day(), 20, 0, 0)
        .single();
    let (lo_ms, hi_ms) = match (lo, hi) {
        (Some(l), Some(h)) => (l.timestamp_millis(), h.timestamp_millis()),
        _ => return latest_ms,
    };
    let hi_ms = hi_ms.min(latest_ms);
    let lo_ms = lo_ms.min(hi_ms);
    let span = (hi_ms - lo_ms).max(0) as f64;
    lo_ms + (rand::random::<f64>() * span) as i64
}

/// 指定日期（days_ago 天前）的 "HH:MM" → 毫秒；未到达则钳制到当前时刻。
fn specified_time(days_ago: i64, time: &str) -> i64 {
    use chrono::{Datelike, TimeZone};
    let (h, m) = time.split_once(':').unwrap_or((time, "0"));
    let (h, m) = (
        h.trim().parse::<u32>().unwrap_or(7) % 24,
        m.trim().parse::<u32>().unwrap_or(0).min(59),
    );
    let base = chrono::Local::now() - chrono::Duration::days(days_ago.clamp(0, 3));
    chrono::Local
        .with_ymd_and_hms(base.year(), base.month(), base.day(), h, m, 0)
        .single()
        .map(|x| x.timestamp_millis())
        .unwrap_or_else(now_ms)
        .min(now_ms())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_minimal_job_with_defaults() {
        let job: RunJob =
            serde_json::from_str(r#"{"username":"u","password":"p"}"#).unwrap();
        assert_eq!(job.username, "u");
        assert_eq!(job.password, "p");
        assert!(job.dist_km.is_none());
        assert!(job.start.is_none());
        assert!(job.device.is_none());
        assert!(job.location.is_none());
    }

    #[test]
    fn parses_full_job() {
        let job: RunJob = serde_json::from_str(
            r#"{
                "username":"13800000000","password":"secret",
                "dist_km":2.5,"pace":"6:30",
                "start":{"mode":"specified","days_ago":1,"time":"08:00"},
                "face_check":false,"altitude_min":30.0,"altitude_max":60.0,"gps_drift_m":2.0,
                "route_mode":"road","custom_close":"roundtrip",
                "custom_datum":"bd09","custom_use_buildings":false,"seed":42,
                "device":{"platform":"android","device_name":"22081212C","os_version":"14"},
                "location":{"city":"北京市","anchor_lat":39.9042,"anchor_lon":116.4074,
                            "auto":true,"offset_m":150.0,"offset_bearing":90.0,"city_auto":true}
            }"#,
        )
        .unwrap();
        assert_eq!(job.dist_km, Some(2.5));
        assert_eq!(job.pace.as_deref(), Some("6:30"));
        let start = job.start.as_ref().unwrap();
        assert_eq!(start.mode, "specified");
        assert_eq!(start.days_ago, 1);
        assert_eq!(start.time, "08:00");
        assert_eq!(job.face_check, Some(false));
        assert_eq!(job.gps_drift_m, Some(2.0));
        assert_eq!(job.seed, Some(42));
        let dev = job.device.as_ref().unwrap();
        assert_eq!(dev.platform.as_deref(), Some("android"));
        let loc = job.location.as_ref().unwrap();
        assert_eq!(loc.city.as_deref(), Some("北京市"));
        assert_eq!(loc.auto, Some(true));
        assert_eq!(loc.offset_bearing, Some(90.0));
        assert_eq!(loc.city_auto, Some(true));
    }

    #[test]
    fn device_and_location_override_identity() {
        let mut identity = HeaderIdentity::default();
        apply_device(
            &mut identity,
            Some(&DeviceSpec {
                platform: Some("android".into()),
                device_name: Some("22081212C".into()),
                ..Default::default()
            }),
        );
        apply_location(
            &mut identity,
            Some(&LocationSpec {
                city: Some("上海市".into()),
                anchor_lat: Some(31.2304),
                anchor_lon: Some(121.4737),
                auto: Some(true),
                offset_m: Some(300.0),
                offset_bearing: Some(45.0),
                city_auto: Some(true),
            }),
        );
        assert_eq!(identity.platform, "android");
        assert_eq!(identity.device_name, "22081212C");
        assert_eq!(identity.city, "上海市");
        assert_eq!(identity.anchor_lat, 31.2304);
        assert!(identity.anchor_auto);
        assert_eq!(identity.anchor_offset_m, 300.0);
        assert_eq!(identity.anchor_offset_bearing, 45.0);
        assert!(identity.city_auto);
        assert!(!identity.has_unconfigured_default_location());
    }

    #[test]
    fn run_config_records_credentials_and_options() {
        let mut cfg = model::Config::default();
        let job: RunJob = serde_json::from_str(
            r#"{"username":"u","password":"p","route_mode":"custom",
                "custom_close":"oneway","custom_datum":"gcj02",
                "custom_use_buildings":false,"amap_key":"K","amap_jscode":"J",
                "gps_drift_m":3.0,"face_check":false}"#,
        )
        .unwrap();
        apply_run_config(&mut cfg, &job);
        assert_eq!(cfg.username, "u");
        assert_eq!(cfg.password, "p");
        assert!(cfg.remember);
        assert_eq!(cfg.route_mode, "custom");
        assert_eq!(cfg.custom_close, "oneway");
        assert_eq!(cfg.custom_datum, "gcj02");
        assert!(!cfg.custom_use_buildings);
        assert_eq!(cfg.amap_key, "K");
        assert_eq!(cfg.amap_security_js_code, "J");
        assert_eq!(cfg.gps_drift_m, 3.0);
        assert!(!cfg.face_check);
    }

    #[test]
    fn parses_range_and_random_start() {
        let job: RunJob = serde_json::from_str(
            r#"{"username":"u","password":"p",
                "dist_km_min":5.0,"dist_km_max":6.0,
                "pace_min":360,"pace_max":480,
                "custom_file":"/tmp/route.txt",
                "start":{"mode":"random","days_ago":1}}"#,
        )
        .unwrap();
        assert_eq!(job.dist_km_min, Some(5.0));
        assert_eq!(job.dist_km_max, Some(6.0));
        assert_eq!(job.pace_min, Some(360.0));
        assert_eq!(job.pace_max, Some(480.0));
        assert_eq!(job.custom_file.as_deref(), Some("/tmp/route.txt"));
        let start = job.start.unwrap();
        assert_eq!(start.mode, "random");
        assert_eq!(start.days_ago, 1);
    }

    /// 随机时刻落在当日 7:00-20:00 且不晚于上界。
    #[test]
    fn random_window_is_bounded_and_within_today() {
        use chrono::{Datelike, TimeZone};
        let now = chrono::Local::now();
        let lo = chrono::Local
            .with_ymd_and_hms(now.year(), now.month(), now.day(), 7, 0, 0)
            .single()
            .unwrap()
            .timestamp_millis();
        let hi = chrono::Local
            .with_ymd_and_hms(now.year(), now.month(), now.day(), 20, 0, 0)
            .single()
            .unwrap()
            .timestamp_millis();
        let latest = now.timestamp_millis();
        for _ in 0..50 {
            let t = random_in_window(0, latest);
            assert!(t >= lo && t <= latest.min(hi), "t={t} lo={lo} hi={hi}");
        }
    }

    /// 上界早于当日窗口起点时退化为上界（凌晨场景不产生未来开跑）。
    #[test]
    fn random_window_clamps_when_window_is_in_the_future() {
        let latest = now_ms() - 86_400_000 - 1_000;
        assert_eq!(random_in_window(0, latest), latest);
    }

    /// 海拔：固定值优先；否则 min/max 组成区间；两值相同视为固定值。
    #[test]
    fn resolves_altitude_from_value_or_range() {
        let cfg = model::Config::default();
        let fixed: RunJob =
            serde_json::from_str(r#"{"altitude":42.5}"#).unwrap();
        let (single, range) = resolve_altitude(&fixed, &cfg).unwrap();
        assert_eq!(single, Some(42.5));
        assert!(range.is_none());

        let ranged: RunJob = serde_json::from_str(
            r#"{"altitude_min":15.0,"altitude_max":25.0}"#,
        )
        .unwrap();
        let (single, range) = resolve_altitude(&ranged, &cfg).unwrap();
        assert!(single.is_none());
        assert_eq!(range.map(|r| (r.min_m, r.max_m)), Some((15.0, 25.0)));

        // 两值相同 = 固定海拔。
        let same: RunJob =
            serde_json::from_str(r#"{"altitude_min":20.0,"altitude_max":20.0}"#).unwrap();
        let (single, range) = resolve_altitude(&same, &cfg).unwrap();
        assert_eq!(single, Some(20.0));
        assert!(range.is_none());

        // 固定值优先于范围。
        let both: RunJob = serde_json::from_str(
            r#"{"altitude":5.0,"altitude_min":15.0,"altitude_max":25.0}"#,
        )
        .unwrap();
        assert_eq!(resolve_altitude(&both, &cfg).unwrap().0, Some(5.0));

        // 都不给 = 沿用本地已保存海拔（默认配置为自动，即 None/None）。
        let none: RunJob = serde_json::from_str(r#"{}"#).unwrap();
        assert_eq!(resolve_altitude(&none, &cfg).unwrap(), (None, None));

        // 本地存有海拔（范围）时，JSON 不给则沿用本地。
        let saved = model::Config {
            manual_altitude_range: Some(crate::track::altitude::AltitudeRange {
                min_m: 11.6,
                max_m: 22.8,
            }),
            ..model::Config::default()
        };
        let inherited = resolve_altitude(&none, &saved).unwrap();
        assert_eq!(
            inherited.1.map(|r| (r.min_m, r.max_m)),
            Some((11.6, 22.8))
        );

        // 显式填了非法区间应报错（而不是静默忽略）。
        let bad: RunJob = serde_json::from_str(r#"{"altitude_min":25.0,"altitude_max":15.0}"#).unwrap();
        assert!(resolve_altitude(&bad, &cfg).is_err());
    }

    /// custom_points 直接内联路径点，生成「纬度,经度」文本。
    #[test]
    fn custom_points_become_route_text() {
        let job: RunJob = serde_json::from_str(
            r#"{"username":"u","password":"p",
                "custom_points":[[39.912179,119.541780],[39.912265,119.541777]]}"#,
        )
        .unwrap();
        assert_eq!(
            inline_route_text(&job).unwrap(),
            "39.912179,119.54178\n39.912265,119.541777"
        );
    }

    /// custom_points 优先于 custom_text 与 custom_file。
    #[test]
    fn custom_points_take_priority() {
        let job: RunJob = serde_json::from_str(
            r#"{"username":"u","password":"p",
                "custom_points":[[1.0,2.0],[3.0,4.0]],
                "custom_text":"9.0,9.0\n8.0,8.0",
                "custom_file":"/nonexistent/should/not/be/read"}"#,
        )
        .unwrap();
        assert_eq!(inline_route_text(&job).unwrap(), "1,2\n3,4");
    }

    /// 点缺少经度时报错而不是静默错位。
    #[test]
    fn custom_points_require_two_numbers() {
        let job: RunJob =
            serde_json::from_str(r#"{"username":"u","password":"p","custom_points":[[1.0]]}"#)
                .unwrap();
        assert!(inline_route_text(&job).is_err());
    }

    #[test]
    fn none_specs_leave_identity_untouched() {
        let mut identity = HeaderIdentity {
            city: "广州市".into(),
            anchor_lat: 23.1291,
            anchor_lon: 113.2644,
            ..Default::default()
        };
        let snapshot = identity.clone();
        apply_device(&mut identity, None);
        apply_location(&mut identity, None);
        assert_eq!(identity.city, snapshot.city);
        assert_eq!(identity.anchor_lat, snapshot.anchor_lat);
        assert_eq!(identity.anchor_lon, snapshot.anchor_lon);
    }
}
