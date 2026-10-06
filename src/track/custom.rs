//! 自定义路径解析：GPX / GeoJSON / 纯文本（每行 `纬度,经度`）+ 坐标基准转换。
//!
//! 支持格式（按内容自动识别）：
//! - **GPX**：`<trkpt lat="" lon="">` 轨迹点；`<rtept>` 路由点；无上述时退化为 `<wpt>` 航点。
//! - **GeoJSON**：`LineString` / `MultiLineString` / `Polygon` 外环 / `Point` 序列
//!   （坐标顺序为 `[lon, lat]`）。
//! - **纯文本**：每行 `纬度,经度`（兼容空格/制表符/分号分隔）；`#` 或 `//` 起首为注释。
//!
//! 坐标基准：导入文件通常为 WGS84（GPS 原始）或 GCJ-02（国内地图拾取），
//! 与服务端打卡点使用的 BD-09 不同系，需按用户选择的基准转换。

use quick_xml::events::{BytesStart, Event};

use super::geom::{gcj02_to_bd09, wgs84_to_bd09};

/// 导入文件的坐标基准。
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum Datum {
    /// WGS84（GPS 原始 / 国际标准，GPX 默认）。
    #[default]
    Wgs84,
    /// GCJ-02（火星坐标，高德/腾讯地图拾取）。
    Gcj02,
    /// BD-09（百度坐标，与服务端打卡点同系）。
    Bd09,
}

impl Datum {
    pub fn from_str(s: &str) -> Self {
        match s.trim().to_ascii_lowercase().as_str() {
            "gcj" | "gcj02" | "gcj-02" | "amap" | "gaode" | "tencent" => Datum::Gcj02,
            "bd" | "bd09" | "bd-09" | "baidu" => Datum::Bd09,
            _ => Datum::Wgs84,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Datum::Gcj02 => "gcj02",
            Datum::Bd09 => "bd09",
            Datum::Wgs84 => "wgs84",
        }
    }

    /// 该基准下的经纬度 → BD-09。
    pub fn to_bd09(self, lat: f64, lon: f64) -> (f64, f64) {
        match self {
            Datum::Wgs84 => wgs84_to_bd09(lat, lon),
            Datum::Gcj02 => gcj02_to_bd09(lat, lon),
            Datum::Bd09 => (lat, lon),
        }
    }
}

/// 解析结果：按输入顺序的 BD 系 (lat, lon) 折线。
#[derive(Clone, Debug)]
pub struct ParsedRoute {
    /// BD-09 坐标序列。
    pub points_bd: Vec<(f64, f64)>,
    /// 识别出的格式名（用于日志展示）。
    pub format: &'static str,
}

/// 解析自定义路径文本（自动识别格式）并转换为 BD-09。
///
/// `datum` 为原始坐标基准；返回的 `points_bd` 已是 BD-09，可直接用于轨迹生成。
pub fn parse_route(text: &str, datum: Datum) -> Result<ParsedRoute, String> {
    let trimmed = text.trim_start_matches('\u{feff}').trim();
    if trimmed.is_empty() {
        return Err("自定义路径内容为空".into());
    }
    let (raw, format) = if trimmed.starts_with('<') {
        parse_xml(trimmed)?
    } else if trimmed.starts_with('{') || trimmed.starts_with('[') {
        parse_geojson(trimmed)?
    } else {
        (parse_text(trimmed), "文本")
    };
    if raw.len() < 2 {
        return Err(format!(
            "解析出 {format} 格式但有效坐标不足 2 个（至少需要起点和另一个点）"
        ));
    }
    let mut points_bd: Vec<(f64, f64)> = Vec::with_capacity(raw.len());
    for &(lat, lon) in &raw {
        let (la, lo) = datum.to_bd09(lat, lon);
        // 相邻重复点（转换后仍重合）去重，避免平滑阶段出现零长段。
        if points_bd
            .last()
            .map(|p| (p.0 - la).abs() < 1e-9 && (p.1 - lo).abs() < 1e-9)
            == Some(true)
        {
            continue;
        }
        points_bd.push((la, lo));
    }
    if points_bd.len() < 2 {
        return Err("坐标去重后不足 2 个点".into());
    }
    Ok(ParsedRoute { points_bd, format })
}

/// XML（GPX）解析：优先 trkpt / rtept，退化为 wpt。
fn parse_xml(text: &str) -> Result<(Vec<(f64, f64)>, &'static str), String> {
    let mut trk: Vec<(f64, f64)> = Vec::new();
    let mut rte: Vec<(f64, f64)> = Vec::new();
    let mut wpt: Vec<(f64, f64)> = Vec::new();
    let mut reader = quick_xml::Reader::from_reader(text.as_bytes());
    reader.config_mut().trim_text(true);
    let mut buf = Vec::new();
    loop {
        buf.clear();
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(e)) | Ok(Event::Empty(e)) => {
                let tag = String::from_utf8_lossy(e.name().as_ref()).into_owned();
                let bucket = match tag.as_str() {
                    "trkpt" => Some(&mut trk),
                    "rtept" => Some(&mut rte),
                    "wpt" => Some(&mut wpt),
                    _ => None,
                };
                if let Some(bucket) = bucket {
                    if let Some(p) = attr_lat_lon(&e) {
                        bucket.push(p);
                    }
                }
            }
            Ok(Event::Eof) => break,
            Err(e) => return Err(format!("GPX 解析失败: {e}")),
            _ => {}
        }
    }
    let (pts, fmt) = if trk.len() >= 2 {
        (trk, "GPX 轨迹")
    } else if rte.len() >= 2 {
        (rte, "GPX 路由")
    } else if wpt.len() >= 2 {
        (wpt, "GPX 航点")
    } else {
        // 单点或空：交给上层报「坐标不足」
        let n = trk.len() + rte.len() + wpt.len();
        let chosen = if !trk.is_empty() {
            trk
        } else if !rte.is_empty() {
            rte
        } else {
            wpt
        };
        if n == 0 {
            return Err("GPX 中未找到 trkpt/rtept/wpt 坐标".into());
        }
        (chosen, "GPX")
    };
    Ok((pts, fmt))
}

/// 从 XML 元素读取 lat / lon 属性（大小写不敏感）。
fn attr_lat_lon(e: &BytesStart<'_>) -> Option<(f64, f64)> {
    let (mut lat, mut lon) = (None, None);
    for attr in e.attributes().with_checks(false).flatten() {
        let key = attr.key.as_ref();
        let val = String::from_utf8_lossy(&attr.value).into_owned();
        match key {
            b"lat" | b"latitude" => lat = val.trim().parse::<f64>().ok(),
            b"lon" | b"lng" | b"longitude" => lon = val.trim().parse::<f64>().ok(),
            _ => {}
        }
    }
    match (lat, lon) {
        (Some(la), Some(lo)) => Some((la, lo)),
        _ => None,
    }
}

/// GeoJSON 解析：LineString / MultiLineString / Polygon / Point / MultiPoint。
///
/// 坐标对为 `[lon, lat]`，与文本的 `纬度,经度` 相反，需转置。
fn parse_geojson(text: &str) -> Result<(Vec<(f64, f64)>, &'static str), String> {
    let v: serde_json::Value =
        serde_json::from_str(text).map_err(|e| format!("GeoJSON 解析失败: {e}"))?;
    let mut pts: Vec<(f64, f64)> = Vec::new();
    collect_geojson(&v, &mut pts);
    if pts.is_empty() {
        return Err("GeoJSON 中未找到 LineString/Polygon/Point 坐标".into());
    }
    Ok((pts, "GeoJSON"))
}

/// 递归收集 GeoJSON 几何坐标（`[lon, lat]` → `(lat, lon)`）。
fn collect_geojson(v: &serde_json::Value, out: &mut Vec<(f64, f64)>) {
    match v {
        serde_json::Value::Array(arr) => {
            // 形如 [lon, lat] 或 [lon, lat, alt] 的坐标对。
            if arr.len() >= 2
                && arr[0].is_number()
                && arr[1].is_number()
                && !arr.iter().any(|x| x.is_array())
            {
                let lon = arr[0].as_f64().unwrap_or(0.0);
                let lat = arr[1].as_f64().unwrap_or(0.0);
                if lat.is_finite() && lon.is_finite() {
                    out.push((lat, lon));
                }
                return;
            }
            for item in arr {
                collect_geojson(item, out);
            }
        }
        serde_json::Value::Object(map) => {
            if let Some(coord) = map.get("coordinates") {
                collect_geojson(coord, out);
            }
            for key in ["geometry", "geometries", "features"] {
                if let Some(child) = map.get(key) {
                    collect_geojson(child, out);
                }
            }
        }
        _ => {}
    }
}

/// 纯文本解析：每行 `纬度,经度`；同一行可用 `;` / `|` 分隔多个点（便于单行输入）。
///
/// 分隔符兼容中文逗号、制表符与空格；`#` / `//` 起首为注释。
fn parse_text(text: &str) -> Vec<(f64, f64)> {
    let mut out = Vec::new();
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') || line.starts_with("//") {
            continue;
        }
        // 一行多点：先按 ; / | 切分
        for chunk in line.split([';', '|']) {
            let normalized = chunk.replace(['，', '\t'], ",");
            let parts: Vec<&str> = normalized
                .split([',', ' '])
                .filter(|s| !s.trim().is_empty())
                .collect();
            if parts.len() < 2 {
                continue;
            }
            let (Ok(lat), Ok(lon)) =
                (parts[0].trim().parse::<f64>(), parts[1].trim().parse::<f64>())
            else {
                continue;
            };
            if lat.is_finite() && lon.is_finite() {
                out.push((lat, lon));
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_plain_text_lines() {
        let text = "# 注释\n38.901678,121.540241\n38.902564,121.541233\n\n# 结束\n";
        let r = parse_route(text, Datum::Wgs84).unwrap();
        assert_eq!(r.points_bd.len(), 2);
        // WGS84 → BD 应带明显偏移
        assert!((r.points_bd[0].0 - 38.901678).abs() > 0.003);
    }

    #[test]
    fn parses_gpx_trkpt() {
        let gpx = r#"<?xml version="1.0"?>
<gpx><trk><trkseg>
<trkpt lat="38.901678" lon="121.540241"/>
<trkpt lat="38.902564" lon="121.541233"/>
<trkpt lat="38.900921" lon="121.542310"/>
</trkseg></trk></gpx>"#;
        let r = parse_route(gpx, Datum::Wgs84).unwrap();
        assert_eq!(r.points_bd.len(), 3);
        assert_eq!(r.format, "GPX 轨迹");
    }

    #[test]
    fn parses_geojson_linestring_lonlat_order() {
        let gj = r#"{"type":"LineString","coordinates":[[121.540241,38.901678],[121.541233,38.902564]]}"#;
        // BD09 基准下不转换，便于核对 lon/lat 顺序是否被正确转置
        let r = parse_route(gj, Datum::Bd09).unwrap();
        assert_eq!(r.points_bd[0], (38.901678, 121.540241));
        assert_eq!(r.points_bd[1], (38.902564, 121.541233));
    }

    #[test]
    fn datum_conversion_differs() {
        let text = "38.901678,121.540241\n38.902564,121.541233";
        let wgs = parse_route(text, Datum::Wgs84).unwrap();
        let bd = parse_route(text, Datum::Bd09).unwrap();
        assert_ne!(wgs.points_bd[0], bd.points_bd[0]);
        assert_eq!(bd.points_bd[0], (38.901678, 121.540241));
    }

    #[test]
    fn rejects_insufficient_points() {
        assert!(parse_route("", Datum::Wgs84).is_err());
        assert!(parse_route("38.9,121.5", Datum::Wgs84).is_err());
    }
}
