//! 轨迹海拔覆盖。
//!
//! 海拔是轨迹点的一部分，而不是提交时临时拼出来的字段。统一在生成器
//! 完成后覆盖 `bdA`，这样提交体的 totalAscent、OBS 的 run_data 和每圈
//! elevationGain 都会读取同一份数据。

#![allow(non_snake_case)]

use super::geom::round_to;
use super::model::{GenPoint, Track};

/// Elevation changes at or below this size are treated as sensor/GPS noise.
pub const ASCENT_NOISE_THRESHOLD_M: f64 = 0.15;

/// 海拔以两位小数存储，相邻差值（如 10.15 - 10.0）会因浮点表示误差
/// 略大于 0.15，这里用 1e-9 容差保证"严格大于 0.15 才计入"的十进制语义。
pub fn positive_ascent_delta(delta_m: f64) -> f64 {
    if delta_m - ASCENT_NOISE_THRESHOLD_M > 1e-9 {
        delta_m
    } else {
        0.0
    }
}

/// Sum positive elevation changes while ignoring small sensor fluctuations.
pub fn total_ascent(locs: &[GenPoint]) -> f64 {
    round_to(
        locs.windows(2)
            .map(|pair| positive_ascent_delta(pair[1].bdA - pair[0].bdA))
            .sum(),
        2,
    )
}

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct AltitudeRange {
    pub min_m: f64,
    pub max_m: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum AltitudeSpec {
    Single(f64),
    Range(AltitudeRange),
}

pub fn parse_spec(text: &str) -> Result<Option<AltitudeSpec>, String> {
    let text = text.trim();
    if text.is_empty() {
        return Ok(None);
    }
    // 先尝试整体解析：允许负海拔单值（如 "-15"），避免负号被误当作区间分隔符。
    if let Ok(v) = text.parse::<f64>() {
        validate_altitude(v)?;
        return Ok(Some(AltitudeSpec::Single(v)));
    }
    if let Some((left, right)) = text.split_once('-') {
        let min_m = left.trim().parse::<f64>().map_err(|_| "手动海拔区间格式应为 min-max，例如 11.6-22.8".to_string())?;
        let max_m = right.trim().parse::<f64>().map_err(|_| "手动海拔区间格式应为 min-max，例如 11.6-22.8".to_string())?;
        return Ok(Some(AltitudeSpec::Range(validate_range(min_m, max_m)?)));
    }
    Err("手动海拔应为数字或 min-max 区间，例如 17.2 或 11.6-22.8".to_string())
}

/// 由两个输入框（最小值 / 最大值）解析海拔。
///
/// 两个都为空 → 不使用覆盖；只填最小值 → 单值绝对海拔；两者都填 → 区间映射。
/// 支持负海拔（低于海平面，如 -15）。
pub fn parse_fields(min_text: &str, max_text: &str) -> Result<Option<AltitudeSpec>, String> {
    let min_t = min_text.trim();
    let max_t = max_text.trim();
    if min_t.is_empty() && max_t.is_empty() {
        return Ok(None);
    }
    if max_t.is_empty() {
        let v = min_t
            .parse::<f64>()
            .map_err(|_| "海拔应为数字".to_string())?;
        validate_altitude(v)?;
        return Ok(Some(AltitudeSpec::Single(v)));
    }
    if min_t.is_empty() {
        return Err("请同时填写海拔区间的最小值".into());
    }
    let min_m = min_t
        .parse::<f64>()
        .map_err(|_| "海拔区间最小值应为数字".to_string())?;
    let max_m = max_t
        .parse::<f64>()
        .map_err(|_| "海拔区间最大值应为数字".to_string())?;
    Ok(Some(AltitudeSpec::Range(validate_range(min_m, max_m)?)))
}

fn validate_altitude(altitude_m: f64) -> Result<(), String> {
    if !altitude_m.is_finite() || !(-500.0..=9000.0).contains(&altitude_m) {
        return Err("海拔必须是 -500 到 9000 米之间的数字（允许负值，如低于海平面）".into());
    }
    Ok(())
}

fn validate_range(min_m: f64, max_m: f64) -> Result<AltitudeRange, String> {
    validate_altitude(min_m)?;
    validate_altitude(max_m)?;
    if min_m > max_m {
        return Err("手动海拔区间的最小值不能大于最大值".into());
    }
    Ok(AltitudeRange { min_m, max_m })
}

/// 将轨迹所有点的百度海拔覆盖为用户输入的绝对海拔（米）。
///
/// 返回 `Err` 而不是静默接受 NaN/无穷值，避免生成不可序列化或被服务端
/// 拒绝的提交。海拔范围采用常见地表范围，既能覆盖地下场地也能覆盖高原。
pub fn override_bd_a(track: &mut Track, altitude_m: f64) -> Result<(), String> {
    validate_altitude(altitude_m)?;
    for point in &mut track.locations {
        point.bdA = round_to(altitude_m, 2);
        point.hasAltitude = true;
    }
    Ok(())
}

/// 将生成器的海拔曲线平滑映射到用户指定的绝对海拔区间。
pub fn override_bd_a_range(track: &mut Track, range: AltitudeRange) -> Result<(), String> {
    let range = validate_range(range.min_m, range.max_m)?;
    let (mut current_min, mut current_max) = (f64::INFINITY, f64::NEG_INFINITY);
    for point in &track.locations {
        current_min = current_min.min(point.bdA);
        current_max = current_max.max(point.bdA);
    }
    let span = current_max - current_min;
    let target_span = range.max_m - range.min_m;
    for point in &mut track.locations {
        let mapped = if span.abs() < f64::EPSILON {
            (range.min_m + range.max_m) / 2.0
        } else {
            range.min_m + ((point.bdA - current_min) / span) * target_span
        };
        point.bdA = round_to(mapped.clamp(range.min_m, range.max_m), 2);
        point.hasAltitude = true;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::api::submit::total_ascent;
    use crate::track::generator::build;
    // 测试统一使用默认漂移距离
    const DRIFT_M: f64 = 1.5;

    fn points() -> Vec<(f64, f64)> {
        vec![(38.901678, 121.540241), (38.902564, 121.541233)]
    }

    #[test]
    fn override_replaces_every_bd_a_and_clears_ascent() {
        let mut track = build(1200.0, 600, 7, (38.9, 121.54), 1_700_000_000_000, &points(), DRIFT_M);
        override_bd_a(&mut track, 36.75).unwrap();
        assert!(track.locations.iter().all(|p| p.bdA == 36.75));
        assert_eq!(total_ascent(&track.locations), 0.0);
    }

    #[test]
    fn rejects_non_finite_or_out_of_range_values() {
        let mut track = build(1000.0, 500, 1, (38.9, 121.54), 1_700_000_000_000, &points(), DRIFT_M);
        assert!(override_bd_a(&mut track, f64::NAN).is_err());
        assert!(override_bd_a(&mut track, 9001.0).is_err());
    }

    #[test]
    fn parses_single_value_and_range() {
        assert_eq!(parse_spec("17.2").unwrap(), Some(AltitudeSpec::Single(17.2)));
        assert_eq!(parse_spec("11.6-22.8").unwrap(), Some(AltitudeSpec::Range(AltitudeRange { min_m: 11.6, max_m: 22.8 })));
        assert!(parse_spec("22.8-11.6").is_err());
    }

    /// 允许负海拔（低于海平面），超出下限才报错。
    #[test]
    fn accepts_negative_altitude() {
        assert_eq!(parse_spec("-15").unwrap(), Some(AltitudeSpec::Single(-15.0)));
        let mut track =
            build(1000.0, 500, 1, (38.9, 121.54), 1_700_000_000_000, &points(), DRIFT_M);
        override_bd_a(&mut track, -15.0).unwrap();
        assert!(track.locations.iter().all(|p| p.bdA == -15.0));
        // 超出下限（-500）仍报错
        assert!(parse_spec("-600").is_err());
    }

    /// 两个输入框解析：空/单值/区间（含负值）。
    #[test]
    fn parses_fields_pair() {
        assert_eq!(parse_fields("", "").unwrap(), None);
        assert_eq!(parse_fields("17.2", "").unwrap(), Some(AltitudeSpec::Single(17.2)));
        assert_eq!(parse_fields("-15", "").unwrap(), Some(AltitudeSpec::Single(-15.0)));
        assert_eq!(
            parse_fields("11.6", "22.8").unwrap(),
            Some(AltitudeSpec::Range(AltitudeRange { min_m: 11.6, max_m: 22.8 }))
        );
        // 只填最大值、区间倒挂应报错
        assert!(parse_fields("", "22.8").is_err());
        assert!(parse_fields("22.8", "11.6").is_err());
    }

    #[test]
    fn range_mapping_stays_inside_requested_bounds() {
        let mut track = build(1200.0, 600, 7, (38.9, 121.54), 1_700_000_000_000, &points(), DRIFT_M);
        override_bd_a_range(&mut track, AltitudeRange { min_m: 11.6, max_m: 22.8 }).unwrap();
        assert!(track.locations.iter().all(|p| (11.6..=22.8).contains(&p.bdA)));
        let (ascent, descent, net) = track.elevation_stats();
        // 噪声过滤下 ascent - descent 不必等于 net，但各项必须有限、非负，
        // 且 net 恒等于末点海拔减首点海拔。
        assert!(ascent.is_finite() && ascent >= 0.0);
        assert!(descent.is_finite() && descent >= 0.0);
        let first = track.locations.first().map(|p| p.bdA).unwrap_or(0.0);
        let last = track.locations.last().map(|p| p.bdA).unwrap_or(0.0);
        assert!((net - (last - first)).abs() < 0.05);
    }

    /// 相邻海拔增量必须严格大于 0.15m 才计入爬升（Issue #38）。
    #[test]
    fn ascent_ignores_small_fluctuations_and_rounds() {
        let mut track = build(1000.0, 500, 1, (38.9, 121.54), 1_700_000_000_000, &points(), DRIFT_M);
        let mut locations = track.locations[..4].to_vec();
        for (point, altitude) in locations.iter_mut().zip([10.0, 10.14, 10.29, 10.45]) {
            point.bdA = altitude;
        }
        track.locations = locations;
        // +0.14 / +0.15 均被忽略，仅 +0.16 计入。
        assert_eq!(total_ascent(&track.locations), 0.16);
        // 恰好 0.15 属于噪声，不计入。
        track.locations[1].bdA = 10.15;
        assert_eq!(total_ascent(&track.locations), 0.16);
    }
}
