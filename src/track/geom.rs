//! 轨迹几何与随机工具。
//!
//! round 封装、RNG、打卡点 Catmull-Rom 拟合环 + 弧长表 + 弧长插值。

use chrono::{Local, TimeZone};
use rand::rngs::StdRng;
use rand::SeedableRng;
use rand_distr::{Distribution, Normal};

pub const MET_PER_DEG_LAT: f64 = 111_132.0;
pub const MET_PER_DEG_LNG: f64 = 86_600.0;

/// round(x, n)：按精确二进制值四舍五入。
pub fn round_to(x: f64, n: usize) -> f64 {
    let s = format!("{x:.n$}");
    s.parse().unwrap_or(x)
}

/// 可播种 RNG 封装。
pub struct Rng {
    inner: StdRng,
}

impl Rng {
    pub fn new(seed: u64) -> Self {
        Self {
            inner: StdRng::seed_from_u64(seed),
        }
    }
    pub fn random(&mut self) -> f64 {
        rand::Rng::gen_range(&mut self.inner, 0.0..1.0)
    }
    pub fn uniform(&mut self, a: f64, b: f64) -> f64 {
        rand::Rng::gen_range(&mut self.inner, a..=b)
    }
    pub fn randint(&mut self, a: i64, b: i64) -> i64 {
        rand::Rng::gen_range(&mut self.inner, a..=b)
    }
    pub fn gauss(&mut self, mu: f64, sigma: f64) -> f64 {
        Normal::new(mu, sigma).unwrap().sample(&mut self.inner)
    }
    pub fn choice<T>(&mut self, items: &[T]) -> T
    where
        T: Copy,
    {
        items[rand::Rng::gen_range(&mut self.inner, 0..items.len())]
    }
    pub fn weighted<T>(&mut self, items: &[(T, u32)]) -> T
    where
        T: Copy,
    {
        let total: u32 = items.iter().map(|(_, w)| *w).sum();
        let mut u = self.uniform(0.0, total as f64);
        for (item, w) in items {
            u -= *w as f64;
            if u < 0.0 {
                return *item;
            }
        }
        items[items.len() - 1].0
    }
}

/// 打卡点 BD 系 (lat, lng) → 闭合平面路径 + 弧长表 + 中心。
pub type PointRing = (Vec<(f64, f64)>, Vec<f64>, (f64, f64));
pub fn make_point_ring(bd_points: &[(f64, f64)]) -> PointRing {
    let n = bd_points.len();
    let cx = bd_points.iter().map(|q| q.0).sum::<f64>() / n as f64;
    let cy = bd_points.iter().map(|q| q.1).sum::<f64>() / n as f64;
    let mut ordered = bd_points.to_vec();
    ordered.sort_by(|a, b| {
        let ka = (a.0 - cx).atan2(a.1 - cy);
        let kb = (b.0 - cx).atan2(b.1 - cy);
        ka.partial_cmp(&kb).unwrap()
    });
    let plane: Vec<(f64, f64)> = ordered
        .iter()
        .map(|q| ((q.1 - cy) * MET_PER_DEG_LNG, (q.0 - cx) * MET_PER_DEG_LAT))
        .collect();
    let samples = 18usize;
    let mut dense = Vec::with_capacity(n * samples);
    for i in 0..n {
        let p0 = plane[(i + n - 1) % n];
        let p1 = plane[i];
        let p2 = plane[(i + 1) % n];
        let p3 = plane[(i + 2) % n];
        for j in 0..samples {
            let t = j as f64 / samples as f64;
            let (t2, t3) = (t * t, t * t * t);
            let x = 0.5
                * ((2.0 * p1.0)
                    + (-p0.0 + p2.0) * t
                    + (2.0 * p0.0 - 5.0 * p1.0 + 4.0 * p2.0 - p3.0) * t2
                    + (-p0.0 + 3.0 * p1.0 - 3.0 * p2.0 + p3.0) * t3);
            let y = 0.5
                * ((2.0 * p1.1)
                    + (-p0.1 + p2.1) * t
                    + (2.0 * p0.1 - 5.0 * p1.1 + 4.0 * p2.1 - p3.1) * t2
                    + (-p0.1 + 3.0 * p1.1 - 3.0 * p2.1 + p3.1) * t3);
            dense.push((x, y));
        }
    }
    let mut arcs = vec![0.0f64];
    for i in 1..=dense.len() {
        let a = dense[i - 1];
        let b = dense[i % dense.len()];
        arcs.push(arcs[i - 1] + ((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2)).sqrt());
    }
    (dense, arcs, (cx, cy))
}

/// 环线弧长 → 坐标（线性插值）。
pub fn ring_point_at(dense: &[(f64, f64)], arcs: &[f64], s: f64) -> (f64, f64) {
    let total = *arcs.last().unwrap_or(&1.0);
    let s = s.rem_euclid(total);
    interpolate_at(dense, arcs, s)
}

/// 折线弧长 → 坐标。
///
/// - `closed`：按总长取模，绕圈重复（适合闭环 / 往返折线）。
/// - 开放（单程）：以三角波在 [0,total] 内折返，避免目标距离超过路径长度时
///   大量采样点堆积在终点（GPS 长时间不动的异常观感）。
pub fn path_point_at(dense: &[(f64, f64)], arcs: &[f64], s: f64, closed: bool) -> (f64, f64) {
    let total = *arcs.last().unwrap_or(&1.0);
    let s = if closed || total <= 0.0 {
        s.rem_euclid(total.max(1e-9))
    } else {
        let period = total * 2.0;
        let x = s.rem_euclid(period);
        if x <= total {
            x
        } else {
            period - x
        }
    };
    interpolate_at(dense, arcs, s)
}

/// 弧长 s（已归一到 [0,total]）→ 坐标（二分定位 + 线性插值）。
fn interpolate_at(dense: &[(f64, f64)], arcs: &[f64], s: f64) -> (f64, f64) {
    let mut lo = 0usize;
    let mut hi = arcs.len();
    while lo < hi {
        let mid = (lo + hi) / 2;
        if arcs[mid] < s {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    let i = lo.max(1).min(dense.len().saturating_sub(1).max(1));
    let a = dense[(i - 1) % dense.len()];
    let b = dense[i % dense.len()];
    let seg = arcs[i] - arcs[i - 1];
    let t = if seg > 0.0 {
        (s - arcs[i - 1]) / seg
    } else {
        0.0
    };
    (a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t)
}

pub fn to_bd(x: f64, y: f64, c_lat: f64, c_lng: f64) -> (f64, f64) {
    (c_lat + y / MET_PER_DEG_LAT, c_lng + x / MET_PER_DEG_LNG)
}

// ── 坐标基准转换：WGS84 ↔ GCJ-02 ↔ BD-09 ─────────────────────────
// 打卡点来自服务端为 BD-09；OSM 路网为 WGS84。真实道路路由需先把路网
// 逐节点转成 BD-09，与经典模式（直接在 BD-09 上拟合打卡点环）保持一致。

const X_PI: f64 = std::f64::consts::PI * 3000.0 / 180.0;
const WGS_A: f64 = 6378245.0;
const WGS_EE: f64 = 0.00669342162296594323;

fn out_of_china(lat: f64, lng: f64) -> bool {
    lng < 72.004 || lng > 137.8347 || lat < 0.8293 || lat > 55.8271
}

fn transform_lat(x: f64, y: f64) -> f64 {
    let pi = std::f64::consts::PI;
    let mut ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * x.abs().sqrt();
    ret += (20.0 * (6.0 * x * pi).sin() + 20.0 * (2.0 * x * pi).sin()) * 2.0 / 3.0;
    ret += (20.0 * (y * pi).sin() + 40.0 * (y / 3.0 * pi).sin()) * 2.0 / 3.0;
    ret += (160.0 * (y / 12.0 * pi).sin() + 320.0 * (y * pi / 30.0).sin()) * 2.0 / 3.0;
    ret
}

fn transform_lng(x: f64, y: f64) -> f64 {
    let pi = std::f64::consts::PI;
    let mut ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * x.abs().sqrt();
    ret += (20.0 * (6.0 * x * pi).sin() + 20.0 * (2.0 * x * pi).sin()) * 2.0 / 3.0;
    ret += (20.0 * (x * pi).sin() + 40.0 * (x / 3.0 * pi).sin()) * 2.0 / 3.0;
    ret += (150.0 * (x / 12.0 * pi).sin() + 300.0 * (x / 30.0 * pi).sin()) * 2.0 / 3.0;
    ret
}

/// WGS84 → GCJ-02（火星坐标，境内偏移）。
pub fn wgs84_to_gcj02(lat: f64, lng: f64) -> (f64, f64) {
    if out_of_china(lat, lng) {
        return (lat, lng);
    }
    let pi = std::f64::consts::PI;
    let mut dlat = transform_lat(lng - 105.0, lat - 35.0);
    let mut dlng = transform_lng(lng - 105.0, lat - 35.0);
    let radlat = lat / 180.0 * pi;
    let mut magic = radlat.sin();
    magic = 1.0 - WGS_EE * magic * magic;
    let sqrtmagic = magic.sqrt();
    dlat = (dlat * 180.0) / ((WGS_A * (1.0 - WGS_EE)) / (magic * sqrtmagic) * pi);
    dlng = (dlng * 180.0) / (WGS_A / sqrtmagic * radlat.cos() * pi);
    (lat + dlat, lng + dlng)
}

/// GCJ-02 → WGS84（`wgs84_to_gcj02` 的近似逆变换，三次迭代收敛到亚米级）。
///
/// OSM/Nominatim 等国际服务要求 WGS-84 入参。
pub fn gcj02_to_wgs84(gcj_lat: f64, gcj_lng: f64) -> (f64, f64) {
    let (mut wlat, mut wlng) = (gcj_lat, gcj_lng);
    for _ in 0..3 {
        let (glat, glng) = wgs84_to_gcj02(wlat, wlng);
        wlat += gcj_lat - glat;
        wlng += gcj_lng - glng;
    }
    (wlat, wlng)
}

/// GCJ-02 → BD-09（`wire::bd09_to_gcj02` 的逆变换）。
pub fn gcj02_to_bd09(gcj_lat: f64, gcj_lng: f64) -> (f64, f64) {
    let z = (gcj_lng * gcj_lng + gcj_lat * gcj_lat).sqrt() + 0.00002 * (gcj_lat * X_PI).sin();
    let theta = gcj_lat.atan2(gcj_lng) + 0.000003 * (gcj_lng * X_PI).cos();
    (z * theta.sin() + 0.006, z * theta.cos() + 0.0065)
}

/// WGS84 → BD-09（OSM 路网对齐打卡点用）。
pub fn wgs84_to_bd09(lat: f64, lng: f64) -> (f64, f64) {
    let (g_lat, g_lng) = wgs84_to_gcj02(lat, lng);
    gcj02_to_bd09(g_lat, g_lng)
}

/// BD-09 坐标按「距离（米）+ 方位角（度）」偏移，返回新的 BD-09 坐标。
///
/// 方位角 0=正北，90=正东，顺时针递增；距离按纬度/经度米-度常量换算，适合数百米级偏移。
pub fn offset_bd(bd_lat: f64, bd_lng: f64, distance_m: f64, bearing_deg: f64) -> (f64, f64) {
    let rad = bearing_deg.to_radians();
    let dlat = distance_m * rad.cos() / MET_PER_DEG_LAT;
    let dlng = distance_m * rad.sin() / MET_PER_DEG_LNG;
    (bd_lat + dlat, bd_lng + dlng)
}

#[cfg(test)]
mod offset_tests {
    use super::*;

    /// GCJ-02 → WGS84 反算后能还原原始 WGS84（迭代逆变换）。
    #[test]
    fn gcj02_to_wgs84_inverts_wgs84_offset() {
        let wgs = (39.9042, 116.4074);
        let (glat, glng) = wgs84_to_gcj02(wgs.0, wgs.1);
        let (blat, blng) = gcj02_to_wgs84(glat, glng);
        assert!((blat - wgs.0).abs() < 1e-6, "lat={blat}");
        assert!((blng - wgs.1).abs() < 1e-6, "lng={blng}");
    }

    /// 正北偏移只改纬度、正东偏移只改经度，且距离换算正确。
    #[test]
    fn offset_bd_moves_along_the_requested_bearing() {
        let (lat, lng) = (39.9, 116.4);
        let (north_lat, north_lng) = offset_bd(lat, lng, 200.0, 0.0);
        assert!((north_lat - lat - 200.0 / MET_PER_DEG_LAT).abs() < 1e-12);
        assert!((north_lng - lng).abs() < 1e-12);

        let (east_lat, east_lng) = offset_bd(lat, lng, 200.0, 90.0);
        assert!((east_lat - lat).abs() < 1e-12);
        assert!((east_lng - lng - 200.0 / MET_PER_DEG_LNG).abs() < 1e-12);
    }
}

pub fn fmt_gain_time(ms: i64) -> String {
    Local
        .timestamp_millis_opt(ms)
        .single()
        .map(|t| t.format("%Y-%m-%d %H:%M:%S").to_string())
        .unwrap_or_default()
}
