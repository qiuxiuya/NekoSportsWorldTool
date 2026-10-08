//! OBS 对象组装。
//!
//! 10 键对象，每值 gzip+base64；BD→GCJ 单次转换；27 键协议点集；
//! 10s 窗（speed/step_freq）；每 1000m 一圈；五点 fixed_point_json。
#![allow(non_snake_case)]

use chrono::{Local, TimeZone};
use flate2::write::GzEncoder;
use flate2::Compression;
use serde_json::{json, Value};
use std::io::Write;

use super::geom::round_to;
use super::model::{GenPoint, Track};

const X_PI: f64 = std::f64::consts::PI * 3000.0 / 180.0;

/// 百度 BD-09 → 高德 GCJ-02。
pub fn bd09_to_gcj02(bd_lat: f64, bd_lng: f64) -> (f64, f64) {
    let x = bd_lng - 0.0065;
    let y = bd_lat - 0.006;
    let z = (x * x + y * y).sqrt() - 0.00002 * (y * X_PI).sin();
    let theta = y.atan2(x) - 0.000003 * (x * X_PI).cos();
    (z * theta.sin(), z * theta.cos())
}

/// gzip + base64。
pub fn gz(data: &[u8]) -> String {
    let mut enc = GzEncoder::new(Vec::new(), Compression::default());
    enc.write_all(data).expect("gzip 写入内存失败");
    let out = enc.finish().expect("gzip 收尾失败");
    super::super::crypto::envelope::b64_encode(&out)
}

fn gz_json(v: &Value) -> String {
    gz(v.to_string().as_bytes())
}

fn gz_str(v: &str) -> String {
    gz(v.as_bytes())
}

/// 27 键协议点集（gen 点 → OBS 点；gLat/gLng 由 BD 转 GCJ）。
pub fn conv_point(p: &GenPoint, start_ms: i64) -> Value {
    let (glat, glng) = bd09_to_gcj02(p.gLat, p.gLng);
    json!({
        "avgSpeed": round_to(p.avgSpeed, 4),
        "bdA": round_to(p.bdA, 2),
        "bdD": round_to(p.bdD, 2),
        "bdG": p.bdG,
        "bdS": round_to(p.bdS, 4),
        "coorType": "gcj02",
        "count": p.count,
        "dtr": 0.0,
        "flag": start_ms,
        "gLat": round_to(glat, 7),
        "gLng": round_to(glng, 7),
        "gainTime": p.gainTime,
        "id": p.id,
        "lat": -1.0,
        "lng": -1.0,
        "locType": p.locType,
        "locationId": "",
        "queueNum": p.queueNum,
        "radius": round_to(p.radius, 2),
        "speed": round_to(p.speed, 4),
        "state": p.state,
        "stepDistance": 0.0,
        "totalDis": round_to(p.totalDis, 4),
        "totalTime": p.totalTime,
        "type": p.ptype,
        "validDis": round_to(p.validDis, 4),
        "validTime": p.validTime,
    })
}

/// 五点实体（实时点位 → 跑完态 isPass=true）。
///
/// 逆向对照（FivePointPlugin / FivePoint 实体）：
/// - position 必须是 1..5 的序号；999 是"非五点"哨兵，会被判定直接跳过；
/// - id 用点位接口返回的真实 id（缺失则退化为 1..5）；
/// - isFixed=1 表示顺序必经点（SEQUENTIAL 策略），随机策略为 0。
pub fn five_point_payload(points: &[Value], start_ms: i64) -> Vec<Value> {
    points
        .iter()
        .enumerate()
        .map(|(i, p)| {
            json!({
                "flag": start_ms,
                "glat": p["glat"].as_f64().unwrap_or(0.0),
                "glon": p["glon"].as_f64().unwrap_or(0.0),
                "id": p["id"].as_i64().unwrap_or(i as i64 + 1),
                "isFixed": p["isFixed"].as_i64().unwrap_or(0),
                "isPass": true,
                "lat": p["lat"].as_f64().unwrap_or(0.0),
                "lon": p["lon"].as_f64().unwrap_or(0.0),
                "pointName": p["pointName"].as_str().unwrap_or(""),
                "position": i as i64 + 1,
                "state": 0,
            })
        })
        .collect()
}

/// record body 的 fivePointJson：官方 UploadFormatEntity 直接放 FivePoint 数组的
/// JSON 字符串（RunUploadHelper: `toJson(list3)`），不是 wrapper。
pub fn five_point_body(points: &[Value], start_ms: i64) -> String {
    Value::Array(five_point_payload(points, start_ms)).to_string()
}

/// OBS fixed_point_json 键的 wrapper（PointJsonEntity 外壳，仅 OBS 通道使用）。
pub fn five_point_wrapper(points: &[Value], start_ms: i64) -> String {
    let five = five_point_payload(points, start_ms);
    json!({
        "useZip": false,
        "fivePointJson": Value::Array(five).to_string(),
        "runAreaId": -1,
        "geoFencesJson": "[]",
        "freedomShowFence": false,
    })
    .to_string()
}

/// 校验提交用五点轨迹仍是同一次点位请求产生的有效数据（record body 直用数组串）。
pub fn validate_five_point_body(body: &str) -> Result<(), String> {
    let points: Vec<Value> =
        serde_json::from_str(body).map_err(|e| format!("五点轨迹数组无效: {e}"))?;
    validate_five_points(&points)
}

/// 校验 OBS fixed_point_json wrapper。
#[allow(dead_code)]
pub fn validate_five_point_wrapper(wrapper: &str) -> Result<(), String> {
    let outer: Value = serde_json::from_str(wrapper).map_err(|e| format!("五点轨迹 JSON 无效: {e}"))?;
    let raw = outer.get("fivePointJson").and_then(Value::as_str).ok_or("五点轨迹缺少 fivePointJson")?;
    let points: Vec<Value> = serde_json::from_str(raw).map_err(|e| format!("五点轨迹数组无效: {e}"))?;
    validate_five_points(&points)
}

fn validate_five_points(points: &[Value]) -> Result<(), String> {
    if points.is_empty() { return Err("五点轨迹不能为空".into()); }
    for (i, point) in points.iter().enumerate() {
        let lat = point.get("lat").and_then(Value::as_f64).unwrap_or(0.0);
        let lon = point.get("lon").and_then(Value::as_f64).unwrap_or(0.0);
        let glat = point.get("glat").and_then(Value::as_f64).unwrap_or(0.0);
        let glon = point.get("glon").and_then(Value::as_f64).unwrap_or(0.0);
        if (lat == 0.0 && lon == 0.0) || (glat == 0.0 && glon == 0.0) { return Err("五点轨迹包含缺失坐标".into()); }
        // position 必须是 1..N 的序号；999 是官方"非五点"哨兵，会被判定跳过
        let pos = point.get("position").and_then(Value::as_i64).unwrap_or(0);
        if pos != i as i64 + 1 { return Err(format!("五点 position 异常: 第{i}点 position={pos}")); }
        crate::location::Coordinate::new(lat, lon, 0.0)?;
        crate::location::Coordinate::new(glat, glon, 0.0)?;
    }
    Ok(())
}

#[cfg(test)]
mod validation_tests {
    use super::*;
    #[test] fn rejects_empty_or_malformed_five_point_payload() {
        assert!(validate_five_point_wrapper("{}").is_err());
        assert!(validate_five_point_wrapper(r#"{"fivePointJson":"[]"}"#).is_err());
        assert!(validate_five_point_body("[]").is_err());
        assert!(validate_five_point_body("not-json").is_err());
        // position=999 哨兵必须被拒绝（官方 FivePointPlugin 遇 999 直接跳过判定）
        let bad = r#"[{"lat":1.0,"lon":2.0,"glat":1.0,"glon":2.0,"position":999}]"#;
        assert!(validate_five_point_body(bad).is_err());
        let good = r#"[{"lat":1.0,"lon":2.0,"glat":1.0,"glon":2.0,"position":1}]"#;
        assert!(validate_five_point_body(good).is_ok());
    }

    #[test]
    fn laps_are_rebuilt_from_overridden_altitude() {
        let points = vec![(38.901678, 121.540241), (38.902564, 121.541233)];
        let mut track = crate::track::generator::build(
            1200.0,
            600,
            7,
            (38.9, 121.54),
            1_700_000_000_000,
            &points,
            1.5,
        );
        crate::track::altitude::override_bd_a(&mut track, 36.75).unwrap();
        let laps = build_laps(&track, track.startTime);
        assert!(!laps.is_empty());
        assert!(laps.iter().all(|lap| lap["elevationGain"] == 0.0));
        assert!(laps.iter().all(|lap| lap["endAltAbs"] == 36.75));
        assert!(laps.iter().all(|lap| lap["endAltRel"] == 0.0));
    }
}

/// 10s 时间窗，id=(rrid%100000)*1000+窗口序秒（6 个真人样本跨 9 月记录验证一致；
/// 旧版 App 样本为全局序号，不适用当前版本）。
fn build_windows(track: &Track, rrid: i64) -> (Vec<Value>, Vec<Value>) {
    let start_ms = track.startTime;
    let total_time = track.totalTime;
    let mut sp = Vec::new();
    let mut stf = Vec::new();
    for (i, a) in track.speedPerTenSec.iter().enumerate() {
        let b = &track.stepsPerTenSec[i];
        let lo = (i * 10) as i64;
        let hi = (i * 10 + 10) as i64;
        let hi = hi.min(total_time);
        let id = (rrid % 100000) * 1000 + hi;
        sp.push(json!({
            "beginTime": start_ms + lo * 1000,
            "distance": a.value,
            "endTime": start_ms + hi * 1000,
            "flag": start_ms,
            "id": id,
            "queueNum": 0,
            "state": 0,
        }));
        stf.push(json!({
            "avgDiff": 0.0,
            "beginTime": start_ms + lo * 1000,
            "endTime": start_ms + hi * 1000,
            "flag": start_ms,
            "id": id,
            "maxDiff": 0.0,
            "minDiff": 1000.0,
            "queueNum": 0,
            "state": 0,
            "stepsNum": b.value as i64,
        }));
    }
    (sp, stf)
}

/// 圈（每 1000m 一圈，末圈 isFullLap=false；avgPace 单位秒/公里，avgStride 单位米）。
fn build_laps(track: &Track, start_ms: i64) -> Vec<Value> {
    let mut laps = Vec::new();
    let locs = &track.locations;
    let (mut prev_d, mut prev_t, mut prev_steps, mut gain, mut loss) =
        (0.0f64, 0i64, 0i64, 0.0f64, 0.0f64);
    let alt0 = locs.first().map(|p| p.bdA).unwrap_or(0.0);
    for (i, pt) in locs.iter().enumerate() {
        if i > 0 {
            let dd = pt.bdA - locs[i - 1].bdA;
            if dd > 0.0 {
                gain += crate::track::altitude::positive_ascent_delta(dd);
            } else {
                loss += -dd;
            }
        }
        let d_now = pt.totalDis;
        let t_now = pt.totalTime;
        let last = i == locs.len() - 1;
        if d_now - prev_d >= 1000.0 || last {
            let lap_d = d_now - prev_d;
            let lap_t = 1i64.max(t_now - prev_t);
            let lap_steps = pt.steps - prev_steps;
            laps.push(json!({
                "avgCadence": round_to(lap_steps as f64 / (lap_t as f64 / 60.0), 2),
                "avgPace": round_to(lap_t as f64 / (lap_d / 1000.0).max(0.001), 2),
                "avgStride": round_to(lap_d / 1.max(lap_steps) as f64, 2),
                "cumulativeDuration": t_now,
                "distance": round_to(lap_d, 4),
                "duration": lap_t,
                "elevationGain": round_to(gain, 2),
                "elevationLoss": round_to(loss, 2),
                "endAltAbs": round_to(pt.bdA, 2),
                "endAltRel": round_to(pt.bdA - alt0, 2),
                "flag": start_ms,
                "id": laps.len() as i64 + 1,
                "isFullLap": lap_d >= 1000.0,
                "lapIndex": laps.len() as i64 + 1,
                "step": lap_steps,
            }));
            prev_d = d_now;
            prev_t = t_now;
            prev_steps = pt.steps;
            gain = 0.0;
            loss = 0.0;
        }
    }
    laps
}

/// 组装 10 键 OBS 对象（值均 gzip+base64；segment_json/runFaceCheck 为空串 gzip）。
///
/// `five_wrapper` 为五点 wrapper 串（PointJsonEntity 外壳）；None 时按 live_points 现拼。
pub fn build_obs_object(
    track: &Track,
    rrid: i64,
    uuid: &str,
    uid: i64,
    live_points: &[Value],
    five_wrapper: Option<&str>,
) -> Value {
    let start_ms = track.startTime;
    let pts: Vec<Value> = track.locations.iter().map(|p| conv_point(p, start_ms)).collect();
    let run_wrap = json!({ "allLocJson": Value::Array(pts).to_string(), "useZip": false });
    let (sp, stf) = build_windows(track, rrid);
    let laps = build_laps(track, start_ms);
    let fx_raw = match five_wrapper {
        Some(w) => w.to_string(),
        None => {
            let five = five_point_payload(live_points, start_ms);
            json!({
                "fivePointJson": Value::Array(five).to_string(),
                "freedomShowFence": false,
                "geoFencesJson": "[]",
                "runAreaId": -1,
                "useZip": false,
            })
            .to_string()
        }
    };
    let fx: Value = serde_json::from_str(&fx_raw).unwrap_or(json!({}));
    json!({
        "rrid": gz_str(&rrid.to_string()),
        "uuid": gz_str(uuid),
        "uid": gz_str(&uid.to_string()),
        "run_data": gz_json(&run_wrap),
        "fixed_point_json": gz_json(&fx),
        "segment_json": gz_str(""),
        "speed_json": gz_json(&Value::Array(sp)),
        "step_freq_json": gz_json(&Value::Array(stf)),
        "laps_json": gz_json(&Value::Array(laps)),
        "runFaceCheck": gz_str(""),
    })
}

/// OBS objectKey（两个都要传）。
pub fn obs_keys(track: &Track, rrid: i64, uuid: &str) -> Vec<String> {
    let t0 = Local
        .timestamp_millis_opt(track.startTime)
        .single()
        .map(|t| t.format("%Y%m%d%H").to_string())
        .unwrap_or_default();
    vec![
        format!("run_data/{t0}/{uuid}.json"),
        format!("run_data/{}/{}.json", rrid / 1000000, rrid),
    ]
}
