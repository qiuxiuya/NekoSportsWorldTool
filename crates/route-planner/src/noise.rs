//! AR(1) 时间自相关红噪声：GPS 漂移（各向异性 + 锚点软回拉）。
//!
//!   e_t = α·e_{t-1} + η_t,  η~N(0, σ_t)
//!
//! σ_t 由建筑 SDF 自适应放大（高楼/遮挡边缘漂移增大）。相关漂移在**走廊局部坐标**
//! 分解为「沿路 / 横向」两轴：沿路方向多普勒/航位推算精度高 → 方差抑制，横向受建筑
//! 遮挡与多路径 → 方差放大；再旋回世界坐标累积，消除各向同性带来的沿路抖动伪影。
//! 接近配置路径点（锚点）时按 OR-OU 指数软回拉，使漂移平滑收敛并保留少量残留误差，
//! 替代事后「一刀切」的坐标覆盖（硬吸附造成的瞬移）。

use rand::rngs::StdRng;
use rand::SeedableRng;

/// 生成 n 步二维 AR(1) 漂移（米），每步标准差由 `sigmas` 给出（SDF 调制后）。
pub fn ar1_xy(alpha: f64, sigmas: &[f64], n: usize, seed: u64) -> Vec<(f64, f64)> {
    use rand_distr::{Distribution, Normal};
    let mut rng = StdRng::seed_from_u64(seed);
    let normal = Normal::<f64>::new(0.0, 1.0).unwrap();
    let mut out = Vec::with_capacity(n);
    let (mut ex, mut ey) = (0.0f64, 0.0f64);
    for i in 0..n {
        let sigma = sigmas.get(i).copied().unwrap_or(0.0);
        ex = alpha * ex + sigma * normal.sample(&mut rng);
        ey = alpha * ey + sigma * normal.sample(&mut rng);
        out.push((ex, ey));
    }
    out
}

fn normalize(v: [f64; 2]) -> [f64; 2] {
    let n = (v[0] * v[0] + v[1] * v[1]).sqrt();
    if n <= 1e-12 {
        [0.0, 0.0]
    } else {
        [v[0] / n, v[1] / n]
    }
}

/// GPS 抖动参数：AR(1) + 走廊各向异性 + 锚点软回拉。
#[derive(Clone, Copy, Debug)]
pub struct JitterParams {
    /// AR(1) 自相关系数 α。
    pub alpha: f64,
    /// 相关漂移的基础标准差（米）。
    pub sigma_correlated: f64,
    /// 独立高斯测量噪声标准差（米）。
    pub sigma_meas: f64,
    /// 沿走廊方向相对基准的方差缩放（<1 抑制，沿路精度高）。
    pub along_ratio: f64,
    /// 垂直走廊方向相对基准的方差缩放（>1 放大，横向受遮挡）。
    pub cross_ratio: f64,
    /// 锚点软回拉尺度（米）；越小回拉越陡。`INFINITY` 表示不回拉。
    pub anchor_sigma: f64,
}

impl Default for JitterParams {
    fn default() -> Self {
        JitterParams {
            alpha: 0.72,
            sigma_correlated: 0.75,
            sigma_meas: 0.3,
            along_ratio: 0.7,
            cross_ratio: 1.3,
            anchor_sigma: 20.0,
        }
    }
}

impl JitterParams {
    /// 由用户「漂移距离」档位（米，约等于相关漂移的稳态幅度）构造参数。
    ///
    /// 相关漂移的基础标准差取 `drift_m × 0.5`（默认 1.5m → 0.75m，与历史行为一致），
    /// 测量噪声取其 0.4 倍；其余各向异性 / 锚点回拉沿用默认。
    pub fn from_drift_m(drift_m: f64) -> Self {
        let sc = (drift_m * 0.5).clamp(0.05, 4.0);
        JitterParams {
            sigma_correlated: sc,
            sigma_meas: sc * 0.4,
            ..JitterParams::default()
        }
    }
}

/// 顺序式 GPS 抖动：AR(1) 相关漂移 + 独立高斯测量噪声。
///
/// 适用于漂移状态需随正常点逐步推进、且方差随 SDF 逐点变化的场景。
/// `step` 的 `scale` 为当前点 SDF 方差放大系数，`z[4]` 为 4 个独立标准正态样本；
/// `tangent` 为行进切线（走廊主轴），`near_anchor_m` 为到最近配置路径点的距离。
pub struct GpsJitter {
    /// 参数。
    pub params: JitterParams,
    ex: f64,
    ey: f64,
}

impl GpsJitter {
    /// 向后兼容构造：等向、无锚点回拉。
    pub fn new(alpha: f64, sigma_correlated: f64, sigma_meas: f64) -> Self {
        Self::with_params(JitterParams {
            alpha,
            sigma_correlated,
            sigma_meas,
            along_ratio: 1.0,
            cross_ratio: 1.0,
            anchor_sigma: f64::INFINITY,
        })
    }

    /// 用完整参数构造（各向异性 + 锚点软回拉）。
    pub fn with_params(params: JitterParams) -> Self {
        GpsJitter {
            params,
            ex: 0.0,
            ey: 0.0,
        }
    }

    /// 当前相关漂移状态（供异常点复用上一次漂移）。
    pub fn state(&self) -> (f64, f64) {
        (self.ex, self.ey)
    }

    /// 硬吸附兜底后同步漂移状态，使后续序列从新位置平滑续接（消除下一跳弹跳）。
    pub fn resync(&mut self, residual: (f64, f64)) {
        self.ex = residual.0;
        self.ey = residual.1;
    }

    /// 前进一步，返回总抖动 (dx, dy)（相关漂移 + 测量噪声）。
    ///
    /// - `scale`：当前点 SDF 方差放大系数；
    /// - `tangent`：行进切线（世界系）；`None` 退化为等向；
    /// - `near_anchor_m`：到最近配置路径点的距离（米），用于 OR-OU 软回拉。
    pub fn step(
        &mut self,
        scale: f64,
        tangent: Option<[f64; 2]>,
        near_anchor_m: f64,
        z: [f64; 4],
    ) -> (f64, f64) {
        let s = self.params.sigma_correlated * scale;
        let (sa, sc) = (s * self.params.along_ratio, s * self.params.cross_ratio);
        // 在走廊局部坐标生成增量，再旋转回世界坐标累积
        let (ix, iy) = match tangent {
            Some(t) => {
                let tn = normalize(t);
                let (nx, ny) = (-tn[1], tn[0]); // 垂直走廊（法向）
                let u = sa * z[0]; // 沿路分量
                let v = sc * z[1]; // 横向分量
                (u * tn[0] + v * nx, u * tn[1] + v * ny)
            }
            None => (s * z[0], s * z[1]),
        };
        self.ex = self.params.alpha * self.ex + ix;
        self.ey = self.params.alpha * self.ey + iy;
        // 锚点软回拉（OR-OU）：越接近配置路径点衰减越强
        if near_anchor_m.is_finite() && self.params.anchor_sigma.is_finite() {
            let d = near_anchor_m.max(0.0);
            let w = (-(d * d) / (2.0 * self.params.anchor_sigma * self.params.anchor_sigma)).exp();
            let k = 1.0 - w;
            self.ex *= k;
            self.ey *= k;
        }
        (
            self.ex + self.params.sigma_meas * z[2],
            self.ey + self.params.sigma_meas * z[3],
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand_distr::{Distribution, Normal};

    /// 生成 n 组独立标准正态样本 z[4]。
    fn samples(n: usize, seed: u64) -> Vec<[f64; 4]> {
        let mut rng = StdRng::seed_from_u64(seed);
        let normal = Normal::<f64>::new(0.0, 1.0).unwrap();
        (0..n)
            .map(|_| {
                [
                    normal.sample(&mut rng),
                    normal.sample(&mut rng),
                    normal.sample(&mut rng),
                    normal.sample(&mut rng),
                ]
            })
            .collect()
    }

    fn std(xs: &[f64]) -> f64 {
        let m = xs.iter().sum::<f64>() / xs.len() as f64;
        (xs.iter().map(|x| (x - m) * (x - m)).sum::<f64>() / xs.len() as f64).sqrt()
    }

    /// 走廊各向异性：沿切线轴方差应显著小于横向。
    #[test]
    fn anisotropic_along_smaller_than_cross() {
        let mut j = GpsJitter::with_params(JitterParams {
            anchor_sigma: f64::INFINITY,
            ..JitterParams::default()
        });
        // 切线沿 x 轴 → 沿路方差落在 dx，横向落在 dy
        let t = Some([1.0, 0.0]);
        let (mut dx, mut dy) = (Vec::new(), Vec::new());
        for z in samples(6000, 42) {
            let (x, y) = j.step(1.0, t, f64::INFINITY, z);
            dx.push(x);
            dy.push(y);
        }
        assert!(
            std(&dx) < std(&dy) * 0.8,
            "沿路方差 {} 未明显小于横向 {}",
            std(&dx),
            std(&dy)
        );
    }

    /// 锚点软回拉：靠近路径点时漂移幅度显著收敛。
    #[test]
    fn anchor_pull_shrinks_drift() {
        let params = JitterParams {
            anchor_sigma: 10.0,
            ..JitterParams::default()
        };
        let t = Some([1.0, 0.0]);
        let zs = samples(200, 7);
        let near = zs.iter().fold(0.0f64, |a, &z| {
            let mut j = GpsJitter::with_params(params);
            let (x, y) = j.step(1.0, t, 0.0, z);
            (x * x + y * y).sqrt().max(a)
        });
        let far = zs.iter().fold(0.0f64, |a, &z| {
            let mut j = GpsJitter::with_params(params);
            let (x, y) = j.step(1.0, t, 1000.0, z);
            (x * x + y * y).sqrt().max(a)
        });
        assert!(near < far, "锚点处漂移 {near} 未小于远处 {far}");
    }

    /// 硬吸附兜底后 resync，状态与残差一致。
    #[test]
    fn resync_sets_state() {
        let mut j = GpsJitter::with_params(JitterParams::default());
        j.resync((1.25, -0.75));
        assert_eq!(j.state(), (1.25, -0.75));
    }
}
