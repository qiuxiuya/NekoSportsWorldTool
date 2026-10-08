//! 跑步提交：POST /api/v70260/runnings/save/record。
//! Android 身份 + runes/runef 头；body 31+ 字段 + signature/originalSign。

use super::client::{check_business, ureq_err, ApiClient};
use super::model::HOST;
use crate::crypto::decrypt::{decrypt_response, derive_paes_key};
use crate::crypto::envelope::{
    build_envelope, build_envelope_ts, rsa_public_key, OuterOrder,
};
use crate::crypto::header::{build_android_header, HeaderIdentity, UA_ANDROID};
use crate::crypto::sign::{original_sign, signature};
use crate::track::calorie::{avg_power, official_kcal};
use crate::track::geom::round_to;
use crate::track::model::{GenPoint, Track};
use crate::track::wire::validate_five_point_wrapper;
use serde_json::{json, Map, Value};

pub const RECORD_PATH: &str = "/api/v70260/runnings/save/record";

/// Android 10s 窗（对齐原项目：id 种子 60000，body 内非空）。
///
/// 原项目把 speedPerTenSec/stepsPerTenSec 作为非空数组提交；`bc922b8` 误改为
/// 空数组。此处回退为原项目行为。
pub fn android_tensec(track: &Track, start_ms: i64, kind: &str) -> Vec<Value> {
    let locs = &track.locations;
    let total_time = track.totalTime;
    let mut out = Vec::new();
    let rid_seed = 60000i64;
    let mut w = 10i64;
    while w <= total_time {
        let lo = w - 10;
        let hi = w.min(total_time);
        let mut d_lo = 0.0f64;
        let mut s_lo = 0i64;
        let mut d_hi = 0.0f64;
        let mut s_hi = 0i64;
        for p in locs {
            let tt = p.totalTime;
            if tt <= lo {
                d_lo = p.totalDis;
                s_lo = p.steps;
            }
            if tt <= hi {
                d_hi = p.totalDis;
                s_hi = p.steps;
            }
        }
        let dist = round_to((d_hi - d_lo).max(0.0), 4);
        let steps_n = (s_hi - s_lo).max(0);
        let begin = start_ms + lo * 1000;
        let end = start_ms + hi * 1000;
        let qn = w / 10 - 1;
        let id = rid_seed + qn;
        if kind == "speed" {
            out.push(json!({
                "beginTime": begin, "distance": dist, "endTime": end,
                "flag": start_ms, "id": id, "queueNum": qn, "state": 0,
            }));
        } else {
            out.push(json!({
                "avgDiff": 0.0, "beginTime": begin, "endTime": end,
                "flag": start_ms, "id": id, "maxDiff": 0.0,
                "minDiff": 1000.0, "queueNum": qn, "state": 0, "stepsNum": steps_n,
            }));
        }
        w += 10;
    }
    out
}

/// bdA 正差分累计，忽略不超过 0.15m 的传感器噪声（与每圈数据共用同一算法）。
pub fn total_ascent(locs: &[GenPoint]) -> f64 {
    crate::track::altitude::total_ascent(locs)
}

/// 累计下降：负差分的绝对值求和。
#[allow(dead_code)]
pub fn total_descent(locs: &[GenPoint]) -> f64 {
    locs.windows(2)
        .map(|pair| (pair[0].bdA - pair[1].bdA).max(0.0))
        .sum()
}

/// 净海拔变化：终点海拔减起点海拔。
#[allow(dead_code)]
pub fn net_elevation_change(locs: &[GenPoint]) -> f64 {
    match (locs.first(), locs.last()) {
        (Some(first), Some(last)) => last.bdA - first.bdA,
        _ => 0.0,
    }
}

fn average_step_frequency(total_steps: i64, total_time: i64) -> i64 {
    (total_steps as f64 / total_time as f64 * 60.0).round() as i64
}

/// 服务端毫配速（不含原项目的 ×1024），保留仅供诊断/明细复用。
#[allow(dead_code)]
pub fn pace_speed_value(total_time_s: i64, distance_m: f64) -> i64 {
    let distance_m = (distance_m * 100.0).ceil() / 100.0;
    if distance_m <= 0.0 {
        return 0;
    }
    (total_time_s as f64 / distance_m * 50.0 / 3.0 * 1000.0).round() as i64
}

pub struct SubmitParams {
    pub track: Track,
    pub uid: i64,
    pub selected_unid: i64,
    pub policy: i64,
    pub policy_ts: i64,
    pub min_distance: i64,
    pub weight: f64,
    pub face_check: i64,
    pub five_point_json: String,
    pub address: String,
}

#[allow(dead_code)]
pub struct SubmitResult {
    pub rrid: i64,
    pub uuid: String,
    pub start_ms: i64,
    pub complete: Option<bool>,
    pub total_dis: f64,
    pub total_time: i64,
    pub total_steps: i64,
    pub avg_step_freq: i64,
    pub calorie: i64,
    pub avg_power: i64,
    pub sel_distance: i64,
}

/// 提交跑步记录（sportType=1 自由跑 + 实时五点）。
pub fn submit_record(client: &mut ApiClient, p: &SubmitParams, log: &mut dyn FnMut(&str)) -> Result<SubmitResult, String> {
    let track = &p.track;
    let start_coordinate = track.validate_consistency()?;
    // 自由跑（无点位学校回退）不传五点；有则必须是有效 wrapper。
    if !p.five_point_json.is_empty() {
        validate_five_point_wrapper(&p.five_point_json)?;
    }
    let total_time = track.totalTime;
    let total_dis = track.totalDistance;
    let total_steps = track.totalSteps;
    let start_ms = track.startTime;
    let stop_ms = start_ms + total_time * 1000;
    let ascent = total_ascent(&track.locations);
    let power = avg_power(p.weight, total_dis, total_time);
    let kcal = official_kcal(p.weight, total_time, total_dis);

    let run_uuid = uuid::Uuid::new_v4().to_string().to_uppercase();
    let unid = p.selected_unid;

    // 原项目：speed = round(时长s / 距离km，2) × 1024（毫配速）
    let dis_ceil = (total_dis * 100.0).ceil() / 100.0;
    let speed = (round_to(total_time as f64 / dis_ceil * 50.0 / 3.0, 2) * 1024.0) as i64;
    let avg_step_freq = 1i64.max(average_step_frequency(total_steps, total_time));

    let mut body = Map::new();
    body.insert("allLocJson".into(), Value::String(String::new()));
    body.insert("sportType".into(), Value::from(1));
    body.insert("policy".into(), Value::from(p.policy));
    body.insert("totalTime".into(), Value::from(total_time));
    body.insert("startTime".into(), Value::from(start_ms));
    body.insert("stopTime".into(), Value::from(stop_ms));
    body.insert("getPrize".into(), Value::Bool(false));
    body.insert("status".into(), Value::from(0));
    body.insert("uuid".into(), Value::String(run_uuid.clone()));
    body.insert("uid".into(), Value::from(p.uid));
    body.insert("selectedUnid".into(), Value::from(unid));
    // 原项目：selRunTime = totalTime
    body.insert("selRunTime".into(), Value::from(total_time));
    body.insert("selDistance".into(), Value::from(p.min_distance));
    body.insert("totalDis".into(), Value::from(round_to(total_dis, 0) as i64));
    body.insert("speed".into(), Value::from(speed));
    body.insert("validDis".into(), Value::from(round_to(total_dis, 0) as i64));
    body.insert("validTime".into(), Value::from(total_time));
    body.insert("complete".into(), Value::Bool(true));
    body.insert("unCompleteReason".into(), Value::from(0));
    body.insert("calorie".into(), Value::from(kcal));
    body.insert("totalSteps".into(), Value::from(total_steps));
    body.insert("avgStepFreq".into(), Value::from(avg_step_freq));
    body.insert("useMobilityTools".into(), Value::from(0));
    body.insert("faceCheck".into(), Value::from(p.face_check));
    // 原项目：totalAscent = round_to(ascent, 0) as i64
    body.insert("totalAscent".into(), Value::from(round_to(ascent, 0) as i64));
    body.insert("avgPower".into(), Value::from(power));
    // 原项目：非空 10s 窗口数组
    body.insert("speedPerTenSec".into(), Value::Array(android_tensec(track, start_ms, "speed")));
    body.insert("stepsPerTenSec".into(), Value::Array(android_tensec(track, start_ms, "steps")));
    body.insert("isUpload".into(), Value::Bool(false));
    body.insert("more".into(), Value::Bool(false));
    body.insert("latitude".into(), Value::from(start_coordinate.latitude));
    body.insert("longitude".into(), Value::from(start_coordinate.longitude));
    body.insert("maxRunTime".into(), Value::from(0));
    body.insert("minSteps".into(), Value::from(0));
    if !p.five_point_json.is_empty() {
        body.insert("fivePointJson".into(), Value::String(p.five_point_json.clone()));
    }
    // Android 必现扩展字段
    body.insert("errorCode".into(), Value::from(0));
    body.insert("geeToken".into(), Value::String(String::new()));
    body.insert("unauthorized".into(), Value::from(0));
    body.insert("themeId".into(), Value::from(0));
    // 原项目：goalId = null（签名串 "null"）
    body.insert("goalId".into(), Value::Null);
    body.insert("address".into(), Value::String(p.address.trim().to_string()));

    let body_val = Value::Object(body.clone());
    let sig = signature(&body_val, false);
    let orig = original_sign(&body_val, false);
    body.insert("signature".into(), Value::String(sig));
    body.insert("originalSign".into(), Value::String(orig));
    let body_plain = Value::Object(body).to_string();

    let device_id = if client.identity.device_id.is_empty() {
        uuid::Uuid::new_v4().to_string().to_uppercase()
    } else {
        client.identity.device_id.clone()
    };
    let android_identity = HeaderIdentity {
        platform: "android".into(),
        device_id: device_id.clone(),
        os_version: "14".into(),
        device_name: "22081212C".into(),
        ..client.identity.clone()
    };
    let (hp, hp_extra) = build_android_header(&android_identity, p.uid, &client_token(client), None);
    let header_env = build_envelope(&mut client.session, &hp, OuterOrder::Observed);
    let now = crate::crypto::envelope::now_ms();
    let body_env = build_envelope_ts(&mut client.session, &body_plain, OuterOrder::Insert, now + 1);

    let runes = format!("{}{}", p.policy_ts, p.uid);
    let runef = format!("{}{}", run_uuid, start_ms);
    let mut req = client.agent.post(&format!("{HOST}{RECORD_PATH}"));
    req = req
        .set("Content-Type", "application/json; charset=utf-8")
        .set("User-Agent", UA_ANDROID)
        .set("appVersion", "7.3.40")
        .set("headerSign", &header_env.json)
        .set("runes", &runes)
        .set("runef", &runef);
    for (k, v) in &hp_extra {
        req = req.set(k, v);
    }
    // 请求明文日志：body（签名后完整业务字段）与 headerSign 明文，便于对照逆向定位 -10000。
    log(&format!("[record] req header={}", crate::textlog::truncate(&hp, 600)));
    log(&format!("[record] req body={}", crate::textlog::truncate(&body_plain, 900)));
    let resp = req.send_string(&body_env.json).map_err(ureq_err)?;
    let status = resp.status();
    let raw = resp.into_string().unwrap_or_default();
    log(&format!("[record] sportType=1 HTTP {status} len={}", raw.len()));

    let key = derive_paes_key(
        &body_env.key_data[0], &body_env.key_data[1], &body_env.key_data[2], &body_env.key_data[3],
    );
    let dec = decrypt_response(raw.as_bytes(), &key, &rsa_public_key())
        .map_err(|e| format!("提交响应解密失败: {e}"))?;
    // 先打印解密后的业务 JSON（成功/失败都要看得到服务端拒绝原因）。
    let mut biz = dec.business.clone();
    log(&format!("[record] resp biz={}", truncate_json(&biz)));
    // -10000「服务器开小差」为服务端内部异常（data=null）；官方 App 是留存本地稍后重传。
    // 客户端做有限次退避重试：每次必须重建 header 明文（含新 timeStamp）与 body 信封，
    // 否则同一 headerSign 明文重复使用会被服务端判 10007。
    if biz.get("error").and_then(|v| v.as_i64()).unwrap_or(0) == -10000 {
        for attempt in 1u64..=2 {
            log(&format!("[record] 服务端异常(-10000)，{attempt}/2 次重试…"));
            std::thread::sleep(std::time::Duration::from_secs(2 * attempt));
            let (retry_hp, retry_hp_extra) =
                build_android_header(&android_identity, p.uid, &client_token(client), None);
            let retry_header = build_envelope(&mut client.session, &retry_hp, OuterOrder::Observed);
            let retry_body = build_envelope_ts(
                &mut client.session,
                &body_plain,
                OuterOrder::Insert,
                crate::crypto::envelope::now_ms() + 1,
            );
            let mut rreq = client.agent.post(&format!("{HOST}{RECORD_PATH}"));
            rreq = rreq
                .set("Content-Type", "application/json; charset=utf-8")
                .set("User-Agent", UA_ANDROID)
                .set("appVersion", "7.3.40")
                .set("headerSign", &retry_header.json)
                .set("runes", &runes)
                .set("runef", &runef);
            for (k, v) in &retry_hp_extra {
                rreq = rreq.set(k, v);
            }
            let Ok(rr) = rreq.send_string(&retry_body.json) else { continue };
            let rs = rr.status();
            let rraw = rr.into_string().unwrap_or_default();
            log(&format!("[record] 重试 HTTP {rs} len={}", rraw.len()));
            let rkey = derive_paes_key(
                &retry_body.key_data[0], &retry_body.key_data[1],
                &retry_body.key_data[2], &retry_body.key_data[3],
            );
            if let Ok(rdec) = decrypt_response(rraw.as_bytes(), &rkey, &rsa_public_key()) {
                log(&format!("[record] 重试 resp biz={}", truncate_json(&rdec.business)));
                if rdec.business.get("error").and_then(|v| v.as_i64()).unwrap_or(0) == 10000 {
                    biz = rdec.business;
                    break;
                }
            }
        }
    }
    let biz = check_business(&biz)?;
    // 响应为 BasicBean<GetDrawChanceBean>：rrid 在 data 内，而 data 可能是 JSON 字符串
    // （与点位/OBS 接口同款包裹），也可能是对象；解析后优先 data.rrid，再兜底顶层 rrid。
    let data = super::client::parse_data_field(&biz);
    let rrid = data
        .get("rrid")
        .and_then(|v| v.as_i64())
        .or_else(|| super::client::get_field(&biz, "rrid").and_then(|v| v.as_i64()))
        .unwrap_or(0);
    if rrid <= 0 {
        return Err(format!("提交未返回 rrid: {}", truncate_json(&biz)));
    }
    log(&format!("√ 提交成功 rrid={rrid} uuid={run_uuid}"));
    Ok(SubmitResult {
        rrid,
        uuid: run_uuid,
        start_ms,
        complete: super::client::get_field(&biz, "complete").and_then(|v| v.as_bool()),
        total_dis,
        total_time,
        total_steps,
        avg_step_freq,
        calorie: kcal,
        avg_power: power,
        sel_distance: p.min_distance,
    })
}

fn client_token(client: &ApiClient) -> String {
    client.login.as_ref().map(|s| s.token.clone()).unwrap_or_default()
}

fn truncate_json(v: &Value) -> String {
    let s = v.to_string();
    s.chars().take(240).collect()
}

#[cfg(test)]
mod tests {
    use super::{
        average_step_frequency, net_elevation_change, pace_speed_value, total_ascent,
        total_descent,
    };

    fn sample_track() -> crate::track::model::Track {
        crate::track::generator::build(
            1000.0,
            374,
            42,
            (38.9, 121.54),
            1_788_958_186_123,
            &[
                (38.901678, 121.540241),
                (38.902564, 121.541233),
                (38.900921, 121.542310),
                (38.899823, 121.541010),
                (38.900455, 121.539512),
            ],
            1.5,
        )
    }

    #[test]
    fn average_step_frequency_uses_rounded_total_steps_over_actual_time() {
        assert_eq!(
            average_step_frequency(773, 374),
            (773.0f64 / 374.0 * 60.0).round() as i64
        );
        assert_eq!(average_step_frequency(7, 8), 53);
    }

    #[test]
    fn elevation_stats_use_positive_and_negative_deltas() {
        let mut track = sample_track();
        let mut points = Vec::new();
        for (i, altitude) in [10.0, 15.0, 12.0, 18.0].iter().copied().enumerate() {
            let mut point = track.locations[0].clone();
            point.totalTime = i as i64 + 1;
            point.bdA = altitude;
            points.push(point);
        }
        track.locations = points;
        // 爬升仅计 +5 与 +6（+0.15 噪声阈值不影响该样本），下降计 -3。
        assert!((total_ascent(&track.locations) - 11.0).abs() < 1e-9);
        assert!((total_descent(&track.locations) - 3.0).abs() < 1e-9);
        assert!(
            (net_elevation_change(&track.locations)
                - (track.locations.last().unwrap().bdA - track.locations.first().unwrap().bdA))
                .abs()
                < 1e-9
        );
    }

    #[test]
    fn pace_uses_thousandths_of_minute_per_kilometre() {
        assert_eq!(pace_speed_value(2295, 5030.0), 7604);
        assert!((pace_speed_value(2295, 5030.0) as f64 / 1000.0 - 7.604).abs() < 0.001);
    }
}
