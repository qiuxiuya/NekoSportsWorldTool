//! 高德步行路径规划（REST API）：按用户给定的路径点，沿真实道路计算步行路线。
//!
//! 借鉴 `map.html` 的 AMap.Walking 思路，但以服务端 REST Web 接口直接调用，
//! 便于桌面端 / Android / CLI 共用（无需内嵌 WebView）：
//!   - 取点：支持「高德地图拾取 / 搜索」得到的 `经度,纬度`（GCJ-02），
//!     可直接粘贴本工程地图导出/复制的坐标文本；
//!   - 逐段请求 `v5/direction/walking`，段间拼接去重，累计真实步行距离；
//!   - 坐标输出统一转 BD-09（与服务端打卡点同系）供轨迹生成使用。
//!
//! 接口：`https://restapi.amap.com/v5/direction/walking`
//! 参数：key / origin(lng,lat) / destination(lng,lat) / show_fields=cost,polyline
//! 若控制台为该 Key 启用了「安全密钥」，需再带 `jscode`（即 securityJsCode）。

#![allow(non_snake_case)]

use serde_json::Value;

use crate::track::custom::Datum;

const AMAP_WALKING_URL: &str = "https://restapi.amap.com/v5/direction/walking";
/// 单段长度上限保护（米）：超长段仍可请求，此处仅用于日志提示。
const STEP_M: f64 = 8.0;

/// 高德步行规划凭据。
#[derive(Clone, Debug)]
pub struct AmapConfig {
    /// Web 服务 Key。
    pub key: String,
    /// 安全密钥 securityJsCode（未启用时留空）。
    pub jscode: String,
}

impl AmapConfig {
    pub fn is_ready(&self) -> bool {
        !self.key.trim().is_empty()
    }
}

/// 高德步行规划结果：BD-09 折线 + 原始 GCJ-02 折线 + 沿路真实长度（米）。
pub struct AmapRoute {
    /// 沿道路的 GCJ-02 折线（供排查坐标基准问题，当前仅内部使用）。
    #[allow(dead_code)]
    pub points_gcj: Vec<(f64, f64)>,
    /// 对应的 BD-09 折线（轨迹生成输入）。
    pub points_bd: Vec<(f64, f64)>,
    /// 沿道路步行的实际总长度（米）。
    pub length_m: f64,
}

/// 逐段调用高德步行规划，拼接为一条折线。
///
/// `seq_gcj` 为完整的 GCJ-02 (lat, lon) 有序点列（至少 2 个）。闭环 / 往返由
/// **调用方**在序列末尾补齐（如闭环追加首点、往返追加逆序回程），本函数只负责
/// 逐段请求、拼接、去重。
///
/// 某段规划失败时退化为该段直线，保证整体可用（与 `map.html` 行为一致）。
pub fn plan_walking(
    cfg: &AmapConfig,
    seq_gcj: &[(f64, f64)],
    log: &mut dyn FnMut(&str),
) -> Result<AmapRoute, String> {
    if !cfg.is_ready() {
        return Err("未配置高德 Key".into());
    }
    let seq: Vec<(f64, f64)> = seq_gcj.to_vec();
    if seq.len() < 2 {
        return Err("高德路径至少需要 2 个点".into());
    }

    let total_segs = seq.len() - 1;
    let mut points_gcj: Vec<(f64, f64)> = Vec::new();
    let mut length_m = 0.0f64;
    let mut ok = 0usize;
    let mut fallback = 0usize;

    for i in 0..total_segs {
        let a = seq[i];
        let b = seq[i + 1];
        log(&format!("[amap] 步行规划 第 {} / {} 段…", i + 1, total_segs));
        match fetch_segment(cfg, a, b) {
            Ok(seg) => {
                length_m += seg.1;
                // 段间首点与上一段末点重合，跳过避免零长段。
                let pts = seg.0;
                let skip = if i > 0 && !points_gcj.is_empty() { 1 } else { 0 };
                for c in pts.into_iter().skip(skip) {
                    push_unique(&mut points_gcj, c);
                }
                ok += 1;
            }
            Err(e) => {
                log(&format!("⚠ [amap] 第 {} 段失败，退化为直线：{e}", i + 1));
                let d = haversine_m(a, b);
                length_m += d;
                for c in densify(a, b, STEP_M) {
                    push_unique(&mut points_gcj, c);
                }
                fallback += 1;
            }
        }
        // 控制请求频率，避免触发 QPS 限制。
        std::thread::sleep(std::time::Duration::from_millis(180));
    }

    if points_gcj.len() < 2 {
        return Err("高德返回的路径点不足".into());
    }
    let points_bd: Vec<(f64, f64)> = points_gcj
        .iter()
        .map(|&(la, lo)| Datum::Gcj02.to_bd09(la, lo))
        .collect();
    log(&format!(
        "√ [amap] 步行路径 {ok} 段成功 / {fallback} 段降级直线，{} 点，约 {:.0} m",
        points_gcj.len(),
        length_m
    ));
    Ok(AmapRoute {
        points_gcj,
        points_bd,
        length_m,
    })
}

/// 请求单个步行段，返回 (GCJ-02 折线, 段长米)。
fn fetch_segment(
    cfg: &AmapConfig,
    a: (f64, f64),
    b: (f64, f64),
) -> Result<(Vec<(f64, f64)>, f64), String> {
    // v5 接口 origin/destination 形如 "经度,纬度"。
    let origin = format!("{:.6},{:.6}", a.1, a.0);
    let dest = format!("{:.6},{:.6}", b.1, b.0);
    let mut url = format!(
        "{AMAP_WALKING_URL}?key={}&origin={}&destination={}&show_fields=polyline,cost",
        url_encode(cfg.key.trim()),
        origin,
        dest
    );
    let js = cfg.jscode.trim();
    if !js.is_empty() {
        url.push_str("&jscode=");
        url.push_str(&url_encode(js));
    }

    let body = http_get(&url)?;
    let v: Value = serde_json::from_str(&body).map_err(|e| format!("高德响应解析失败: {e}"))?;
    let status = v.get("status").and_then(|s| s.as_str()).unwrap_or("");
    if status != "1" {
        let info = v.get("info").and_then(|s| s.as_str()).unwrap_or("未知错误");
        let code = v.get("infocode").and_then(|s| s.as_str()).unwrap_or("");
        return Err(format!("高德返回 {info}（{code}）"));
    }

    let path = v
        .pointer("/route/paths/0")
        .ok_or("高德响应缺少 route.paths[0]")?;
    let distance = path
        .get("distance")
        .and_then(|d| d.as_str())
        .and_then(|s| s.parse::<f64>().ok())
        .unwrap_or(0.0);
    let mut pts: Vec<(f64, f64)> = Vec::new();
    if let Some(steps) = path.get("steps").and_then(|s| s.as_array()) {
        for step in steps {
            let Some(poly) = step.get("polyline").and_then(|p| p.as_str()) else {
                continue;
            };
            for pair in poly.split(';') {
                if let Some((lo, la)) = pair.split_once(',') {
                    if let (Ok(lon), Ok(lat)) = (lo.trim().parse::<f64>(), la.trim().parse::<f64>())
                    {
                        push_unique(&mut pts, (lat, lon));
                    }
                }
            }
        }
    }
    if pts.is_empty() {
        // 无 polyline（部分精简返回）：退化为直线端点。
        pts.push(a);
        pts.push(b);
    }
    let len = if distance > 0.0 {
        distance
    } else {
        pts.windows(2).map(|w| haversine_m(w[0], w[1])).sum()
    };
    Ok((pts, len))
}

/// 走信封无关的裸 GET：返回响应体文本（高德 REST 不走业务信封）。
fn http_get(url: &str) -> Result<String, String> {
    let resp = ureq::get(url)
        .timeout(std::time::Duration::from_secs(20))
        .call()
        .map_err(|e| format!("高德请求失败: {e}"))?;
    resp.into_string()
        .map_err(|e| format!("高德响应读取失败: {e}"))
}

fn push_unique(arr: &mut Vec<(f64, f64)>, p: (f64, f64)) {
    if let Some(last) = arr.last() {
        if (last.0 - p.0).abs() < 1e-9 && (last.1 - p.1).abs() < 1e-9 {
            return;
        }
    }
    arr.push(p);
}

/// 两点直线距离（米，经纬度为度）。
fn haversine_m(a: (f64, f64), b: (f64, f64)) -> f64 {
    let r = 6_371_000.0f64;
    let dlat = (b.0 - a.0).to_radians();
    let dlng = (b.1 - a.1).to_radians();
    let la1 = a.0.to_radians();
    let la2 = b.0.to_radians();
    let h = (dlat / 2.0).sin().powi(2) + la1.cos() * la2.cos() * (dlng / 2.0).sin().powi(2);
    2.0 * r * h.sqrt().min(1.0).asin()
}

/// 直线加密为间距约 `step_m` 的点列（含起点，不含终点）。
fn densify(a: (f64, f64), b: (f64, f64), step_m: f64) -> Vec<(f64, f64)> {
    let d = haversine_m(a, b);
    if d < 0.5 {
        return vec![a];
    }
    let n = ((d / step_m).ceil() as usize).max(1);
    (0..n)
        .map(|k| {
            let t = k as f64 / n as f64;
            (a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t)
        })
        .collect()
}

/// 极简 URL 编码（key / jscode 可能含 `+ / =` 等）。
fn url_encode(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for byte in s.bytes() {
        match byte {
            b'A'..=b'Z' | b'a'..=b'z' | b'0'..=b'9' | b'-' | b'_' | b'.' | b'~' => {
                out.push(byte as char)
            }
            _ => out.push_str(&format!("%{byte:02X}")),
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn url_encode_escapes_reserved() {
        assert_eq!(url_encode("a+b/c=d"), "a%2Bb%2Fc%3Dd");
        assert_eq!(url_encode("Abc123-_.~"), "Abc123-_.~");
    }

    #[test]
    fn densify_spacing_and_endpoints() {
        // 约 111m 的纬度跨度，8m 间距 → 约 14 个点（不含终点）
        let pts = densify((38.9, 121.5), (38.901, 121.5), 8.0);
        assert!(!pts.is_empty());
        assert_eq!(pts[0], (38.9, 121.5));
        assert!(pts.len() >= 12 && pts.len() <= 16, "n={}", pts.len());
    }

    #[test]
    fn haversine_basic() {
        let d = haversine_m((38.9, 121.5), (38.901, 121.5));
        assert!((d - 111.2).abs() < 2.0, "d={d}");
    }
}
