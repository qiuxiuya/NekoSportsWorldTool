//! 校园点位：POST /api/v560/get/1/distance/1（sportType=4）。
//!
//! runec = 信封(f"{uid}{经度6位}{纬度6位}{起始时间秒级ms整}"，insert→observed 序列化)；
//! sign = MD5(http版URL + 盐)。带 300s 缓存（限流 10603：5 分钟 3 次），失败回退最近缓存。

use super::client::ApiClient;
use super::model;
use crate::crypto::envelope::{build_envelope, OuterOrder};
use crate::crypto::sign::md5_url_sign;
use crate::location::Coordinate;
use serde_json::{json, Value};

pub const POINTS_PATH: &str = "/api/v560/get/1/distance/1";

/// 点位 + 服务端区域元数据（详情页绿色围栏/目标点来源）。
#[derive(Clone, Debug)]
pub struct PointsContext {
    pub points: Vec<Value>,
    pub area: crate::track::wire::RunAreaMeta,
    /// 服务端明确返回"学校未设置点位"（error=10600）：调用方应回退自由跑。
    pub no_points: bool,
}

/// 经纬度六位小数字符串。
fn six_digit(v: f64) -> String {
    format!("{v:.6}")
}

/// 拉取实时点位（带缓存回退）。anchor=(lat,lng) 请求锚点。
pub fn fetch_points(
    client: &mut ApiClient,
    anchor: Coordinate,
    log: &mut dyn FnMut(&str),
) -> Result<Vec<Value>, String> {
    Ok(fetch_points_context(client, anchor, log)?.points)
}

/// 拉取点位 + 区域元数据。
pub fn fetch_points_context(
    client: &mut ApiClient,
    anchor: Coordinate,
    log: &mut dyn FnMut(&str),
) -> Result<PointsContext, String> {
    fetch_points_context_ext(client, anchor, None, log)
}

/// run_area_id：学校配置了区域时 App 会附带；未配置则不传（与 App 一致）。
#[allow(dead_code)]
pub fn fetch_points_ext(
    client: &mut ApiClient,
    anchor: Coordinate,
    run_area_id: Option<String>,
    log: &mut dyn FnMut(&str),
) -> Result<Vec<Value>, String> {
    Ok(fetch_points_context_ext(client, anchor, run_area_id, log)?.points)
}

pub fn fetch_points_context_ext(
    client: &mut ApiClient,
    anchor: Coordinate,
    run_area_id: Option<String>,
    log: &mut dyn FnMut(&str),
) -> Result<PointsContext, String> {
    anchor.validate()?;
    // ① TTL 内命中缓存直接返回（同时带出区域元数据）
    if let Some((ts, pts, area)) = model::load_points_cache_context_for(anchor) {
        if !pts.is_empty()
            && crate::crypto::envelope::now_ms() - ts < model::POINTS_TTL_MS
        {
            log(&format!("[points] 缓存命中（{} 秒前，{} 点，围栏={}）", (crate::crypto::envelope::now_ms() - ts) / 1000, pts.len(), area.freedom_show_fence));
            return Ok(PointsContext { points: pts, area, no_points: false });
        }
    }
    // ② 请求接口
    let uid = client.login.as_ref().map(|s| s.uid).unwrap_or(0);
    let unid = client
        .login
        .as_ref()
        .map(|s| s.unid.clone())
        .unwrap_or_else(|| "0".into());
    let lat = anchor.latitude;
    let lon = anchor.longitude;
    let url = format!("{}{}", model::HOST, POINTS_PATH);

    let start_ms = crate::crypto::envelope::now_ms();
    let runec_input = format!("{uid}{}{}{}", six_digit(lon), six_digit(lat), (start_ms / 1000) * 1000);
    // runec：共用会话 Four，外层 observed 序
    let runec_env = build_envelope(&mut client.session, &runec_input, OuterOrder::Observed);
    let runec = runec_env.json;

    let mut body = json!({
        "sportType": 4,
        "longitude": lon,
        "latitude": lat,
        "sign": md5_url_sign(&url),
        "uuid": uuid::Uuid::new_v4().to_string(),
        "selectedUnid": unid,
        "runec": runec,
    });
    if let Some(area) = run_area_id {
        body["runAreaId"] = json!(area);
    }
    let body = body.to_string();

    let out = client.envelope_request("POST", &url, &body, crate::crypto::header::UA_IOS, &[])?;
    let fallback = |log: &mut dyn FnMut(&str)| -> Result<PointsContext, String> {
        if let Some((_ts, pts, area)) = model::load_points_cache_context_for(anchor) {
            if !pts.is_empty() {
                log("[points] 接口失败，回退最近一次接口结果缓存");
                return Ok(PointsContext { points: pts, area, no_points: false });
            }
        }
        Err("点位接口失败且无缓存".into())
    };
    let Some(dec) = out.decrypted else {
        return fallback(log);
    };
    let payload = &dec.business;
    // pointsResModels 可能位于 data/result 或编码成 JSON 字符串；多字段名回退
    let pts = extract_points(payload);
    if !pts.is_empty() {
        let area = area_from_payload(payload, &pts);
        let _ = model::save_points_cache_context(anchor, &pts, &area);
        return Ok(PointsContext { points: pts, area, no_points: false });
    }
    let err = payload.get("error").and_then(|e| e.as_i64()).unwrap_or(0);
    let msg = payload.get("message").and_then(|m| m.as_str()).unwrap_or("");
    log(&format!("[points] 接口无点位 error={err}: {msg}"));
    // 10600 = 学校未设置点位：不视为错误，交由调用方回退自由跑
    if err == 10600 {
        let area = area_from_payload(payload, &[]);
        return Ok(PointsContext { points: Vec::new(), area, no_points: true });
    }
    fallback(log)
}

// ── 点位 / 区域（围栏）提取 ───────────────────────────────────
// 接口版本之间字段层级不同，必须递归查找（含 data/result、JSON 字符串），
// 否则围栏被丢，详情页只显示灰色轨迹且无目标点。

fn extract_points(payload: &Value) -> Vec<Value> {
    let names = ["pointsResModels", "pointResModels", "points", "pointList", "pointsModelList"];
    find_value_recursive(payload, &names, 8)
        .and_then(|value| match value {
            Value::Array(items) => Some(items),
            Value::String(text) => serde_json::from_str::<Value>(&text).ok().and_then(|v| v.as_array().cloned()),
            _ => None,
        })
        .map(|items| items.into_iter().map(normalize_point).collect())
        .unwrap_or_default()
}

/// 把服务端点位的别名字段归一化到规范名（id/position/state/coorType）。
fn normalize_point(mut point: Value) -> Value {
    let Some(object) = point.as_object_mut() else { return point };
    for (canonical, aliases) in [
        ("id", ["id", "pointId", "pointID", "fixedPointId", "fixedPointID", "checkpointId", "checkPointId"].as_slice()),
        ("position", ["position", "pointPosition", "pointIndex", "sort", "sortNum", "seq", "sequence", "order", "orderNum"].as_slice()),
        ("state", ["state", "pointState", "status", "pointStatus", "passState"].as_slice()),
        ("coorType", ["coorType", "coordType", "coordinateType"].as_slice()),
    ] {
        if object.get(canonical).is_some_and(|v| !v.is_null()) {
            continue;
        }
        if let Some(value) = aliases.iter().find_map(|a| object.get(*a).filter(|v| !v.is_null())).cloned() {
            object.insert(canonical.into(), value);
        }
    }
    point
}

fn find_value_recursive(root: &Value, names: &[&str], depth: usize) -> Option<Value> {
    if depth == 0 { return None; }
    match root {
        Value::Object(map) => {
            for name in names {
                if let Some(v) = map.get(*name).filter(|v| !v.is_null()) { return Some(v.clone()); }
            }
            for v in map.values() {
                if let Some(found) = find_value_recursive(v, names, depth - 1) { return Some(found); }
            }
        }
        Value::Array(items) => {
            for v in items {
                if let Some(found) = find_value_recursive(v, names, depth - 1) { return Some(found); }
            }
        }
        Value::String(text) => {
            if let Ok(parsed) = serde_json::from_str::<Value>(text) {
                return find_value_recursive(&parsed, names, depth - 1);
            }
        }
        _ => {}
    }
    None
}

fn find_usable_field(root: &Value, names: &[&str], depth: usize) -> Option<Value> {
    if depth == 0 { return None; }
    match root {
        Value::Object(map) => {
            for name in names {
                if let Some(v) = map.get(*name).filter(|v| usable_fence(v)) { return Some(v.clone()); }
            }
            for v in map.values() {
                if let Some(found) = find_usable_field(v, names, depth - 1) { return Some(found); }
            }
        }
        Value::Array(items) => {
            for v in items {
                if let Some(found) = find_usable_field(v, names, depth - 1) { return Some(found); }
            }
        }
        Value::String(text) => {
            if let Ok(parsed) = serde_json::from_str::<Value>(text) {
                return find_usable_field(&parsed, names, depth - 1);
            }
        }
        _ => {}
    }
    None
}

fn find_true_field(root: &Value, names: &[&str], depth: usize) -> Option<Value> {
    if depth == 0 { return None; }
    match root {
        Value::Object(map) => {
            for name in names {
                if let Some(v) = map.get(*name).filter(|v| value_as_bool(v) == Some(true)) { return Some(v.clone()); }
            }
            for v in map.values() {
                if let Some(found) = find_true_field(v, names, depth - 1) { return Some(found); }
            }
        }
        Value::Array(items) => {
            for v in items {
                if let Some(found) = find_true_field(v, names, depth - 1) { return Some(found); }
            }
        }
        Value::String(text) => {
            if let Ok(parsed) = serde_json::from_str::<Value>(text) {
                return find_true_field(&parsed, names, depth - 1);
            }
        }
        _ => {}
    }
    None
}

fn find_nonnegative_field(root: &Value, names: &[&str], depth: usize) -> Option<i64> {
    if depth == 0 { return None; }
    match root {
        Value::Object(map) => {
            for name in names {
                if let Some(id) = map.get(*name).and_then(value_as_i64).filter(|id| *id >= 0) { return Some(id); }
            }
            for v in map.values() {
                if let Some(id) = find_nonnegative_field(v, names, depth - 1) { return Some(id); }
            }
        }
        Value::Array(items) => {
            for v in items {
                if let Some(id) = find_nonnegative_field(v, names, depth - 1) { return Some(id); }
            }
        }
        Value::String(text) => {
            if let Ok(parsed) = serde_json::from_str::<Value>(text) {
                return find_nonnegative_field(&parsed, names, depth - 1);
            }
        }
        _ => {}
    }
    None
}

fn value_as_i64(value: &Value) -> Option<i64> {
    value
        .as_i64()
        .or_else(|| value.as_u64().and_then(|n| i64::try_from(n).ok()))
        .or_else(|| value.as_f64().filter(|n| n.is_finite()).map(|n| n as i64))
        .or_else(|| value.as_str().and_then(|t| t.trim().parse().ok()))
        .or_else(|| value.get("id").and_then(value_as_i64))
        .or_else(|| value.get("runAreaId").and_then(value_as_i64))
}

fn value_as_bool(value: &Value) -> Option<bool> {
    value
        .as_bool()
        .or_else(|| value.as_i64().map(|n| n != 0))
        .or_else(|| value.as_str().and_then(|s| match s.trim().to_ascii_lowercase().as_str() {
            "true" | "1" | "yes" => Some(true),
            "false" | "0" | "no" => Some(false),
            _ => None,
        }))
}

fn usable_fence(value: &Value) -> bool {
    let text = value_as_json_string(value);
    let Ok(parsed) = serde_json::from_str::<Value>(text.trim()) else { return false };
    matches!(parsed, Value::Array(ref items) if !items.is_empty())
}

fn value_as_json_string(value: &Value) -> String {
    match value {
        Value::String(text) => serde_json::from_str::<Value>(text).map(|v| v.to_string()).unwrap_or_else(|_| text.clone()),
        Value::Null => "[]".into(),
        other => other.to_string(),
    }
}

fn fence_id(value: &Value) -> Option<i64> {
    let parsed = match value {
        Value::String(text) => serde_json::from_str::<Value>(text).ok()?,
        other => other.clone(),
    };
    let items = parsed.as_array()?;
    items.iter().find_map(|item| {
        item.get("runAreaId").or_else(|| item.get("areaId")).or_else(|| item.get("id"))
            .and_then(value_as_i64)
            .filter(|id| *id >= 0)
    })
}

/// 从点位/策略响应提取运行区域元数据（递归多字段名，兼容字符串/嵌套）。
pub(crate) fn area_from_payload(payload: &Value, points: &[Value]) -> crate::track::wire::RunAreaMeta {
    let id_names = ["runAreaId", "runAreaID", "areaId", "areaID"];
    let fence_names = [
        "geoFencesJson", "geoFenceJson", "geoFences", "geoFence", "geoFenceList",
        "fenceList", "fences", "runAreaGeoFences", "runAreaFence",
    ];
    let show_names = ["freedomShowFence", "showFence", "showGeoFence", "isShowFence"];
    // 优先非负 id：有些响应顶层是默认 -1，真实 id 在 runArea/runAreaInfo 内
    let mut run_area_id = find_nonnegative_field(payload, &id_names, 8)
        .or_else(|| find_nonnegative_field(payload, &["runArea", "runAreaInfo"], 8))
        .or_else(|| find_nonnegative_field(payload, &["runId"], 8));
    let mut fences = find_usable_field(payload, &fence_names, 8);
    let mut show = find_true_field(payload, &show_names, 8)
        .or_else(|| find_value_recursive(payload, &show_names, 8));
    for point in points {
        if run_area_id.is_none() {
            run_area_id = find_nonnegative_field(point, &id_names, 3)
                .or_else(|| find_nonnegative_field(point, &["runArea", "runAreaInfo"], 3));
        }
        if fences.is_none() {
            fences = find_usable_field(point, &fence_names, 3);
        }
        if show.is_none() {
            show = find_true_field(point, &show_names, 3)
                .or_else(|| find_value_recursive(point, &show_names, 3));
        }
    }
    // 校园响应常把区域 id 放在围栏对象上而非独立 runAreaId 字段，保留它
    let run_area_id = run_area_id.or_else(|| fences.as_ref().and_then(fence_id)).unwrap_or(-1);
    let geo_fences_json = fences
        .as_ref()
        .map(value_as_json_string)
        .filter(|v| !v.trim().is_empty() && v.trim() != "null" && v.trim() != "[]")
        .unwrap_or_else(|| "[]".into());
    let freedom_show_fence = show
        .as_ref()
        .and_then(value_as_bool)
        .unwrap_or(geo_fences_json.trim() != "[]");
    crate::track::wire::RunAreaMeta { run_area_id, geo_fences_json, freedom_show_fence }
}

/// 点位中心（BD 系）。
#[allow(dead_code)]
pub fn center_bd(points: &[Value]) -> (f64, f64) {
    let n = points.len().max(1) as f64;
    let lat = points.iter().filter_map(|p| p.get("lat").and_then(|v| v.as_f64())).sum::<f64>() / n;
    let lon = points.iter().filter_map(|p| p.get("lon").and_then(|v| v.as_f64())).sum::<f64>() / n;
    (lat, lon)
}

/// 点位 → (lat, lon) BD 系数组（轨迹输入）。
pub fn points_bd(points: &[Value]) -> Vec<(f64, f64)> {
    points
        .iter()
        .filter_map(|p| {
            let lat = p.get("lat")?.as_f64()?;
            let lon = p.get("lon")?.as_f64()?;
            Some((lat, lon))
        })
        .collect()
}
