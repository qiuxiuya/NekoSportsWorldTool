//! 真实道路拓扑路由 + 运动学物理仿真库。
//!
//! 三层解耦：
//!   1. 路由层（纯几何/拓扑）：OSM 解析 → 有向图 → 虚拟节点投影切分 → 闭环路线
//!   2. 动力学层（运动学 + 传感器）：曲率限速 / OU 配速 / AR(1) 噪声 + SDF / 生物力学
//!   3. 协议层（调用方适配）：由 `Route` + 速度序列组装最终轨迹
//!
//! 坐标约定：本 crate 内部以「度」存储（调用方已把 OSM 对齐到 BD 工作系），
//! 距离/曲率经本地等距投影转米制平面。

use petgraph::graph::NodeIndex;

pub mod biomech;
pub mod graph;
pub mod kinematics;
pub mod load;
pub mod noise;
pub mod project;
pub mod route;
pub mod sdf;
pub mod smooth;
pub mod speed;

pub use biomech::{cadence_for, fatigue_from_km, gait, ou_cadence_series, speed_from, stride_for};
pub use graph::{point_in_polygon, Coord, RoadGraph};
pub use kinematics::{pace_profile, speed_limit_ahead, KinParams};
pub use load::{load_osm, parse_osm};
pub use noise::{ar1_xy, GpsJitter, JitterParams};
pub use route::RouteOptions;
pub use sdf::Sdf;
pub use smooth::{turn_speed_limit, RoutePoint};

pub use self::CloseMode as RouteCloseMode;

/// 一条规划好的空间路线（度系采样点 + 总长）。
#[derive(Clone, Debug)]
pub struct Route {
    pub points: Vec<RoutePoint>,
    pub length_m: f64,
}

/// 入口：解析路网 + 虚拟节点 + 闭环路由 + 曲率平滑。
///
/// `waypoints[0]` 为起点，`waypoints[1..]` 为必经打卡点（按顺序），
/// 路线自动闭合回起点。
pub fn plan_route(
    net: &RoadGraph,
    waypoints: &[Coord],
    target_len_m: f64,
    seed: u64,
    opts: &RouteOptions,
) -> Result<Route, String> {
    plan_route_split(net, waypoints, &waypoints[1..], target_len_m, seed, opts)
}

/// 首尾走法（自定义路径 / 高德路径共用）。
///
/// - `Closed`：首尾相连成环（默认，跑圈）。
/// - `RoundTrip`：原路返回（去程 + 回程，同一段路走两遍）。
/// - `OneWay`：单程，仅从起点跑到终点，不回起点。
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum CloseMode {
    /// 首尾相连成环。
    #[default]
    Closed,
    /// 原路返回（往返）。
    RoundTrip,
    /// 单程（不回起点）。
    OneWay,
}

impl CloseMode {
    pub fn from_str(s: &str) -> Self {
        match s.trim().to_ascii_lowercase().as_str() {
            "roundtrip" | "round-trip" | "return" | "往返" => CloseMode::RoundTrip,
            "oneway" | "one-way" | "single" | "单程" => CloseMode::OneWay,
            _ => CloseMode::Closed,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            CloseMode::Closed => "closed",
            CloseMode::RoundTrip => "roundtrip",
            CloseMode::OneWay => "oneway",
        }
    }

    /// 是否回到起点（闭环 / 往返为真，单程为假）。
    pub fn is_closed(self) -> bool {
        !matches!(self, CloseMode::OneWay)
    }
}

/// 入口（自定义折线）：按用户给定的经纬度顺序构建路线，**不做最短路径重排**。
///
/// 与 `plan_route` 的区别：几何源为折线本身而非路网拓扑，因此即使没有 OSM 路网
/// 也能规划。行为：
///   1. 过滤相邻重复点；
///   2. 按 `close` 拼接走法：`Closed` 在末尾补回起点（闭合）；`RoundTrip` 追加逆序
///      回程（原路返回）；`OneWay` 保持单程不回起点（`Route.points` 为开放折线）；
///   3. 用给定切角半径 + 阈值对转角做圆弧平滑并按 `step_m` 重采样。
///
/// `threshold_deg` 为低于该角度的转角不切（保持直线）。
pub fn plan_route_polyline(
    waypoints: &[Coord],
    close: CloseMode,
    radius_m: f64,
    threshold_deg: f64,
    step_m: f64,
) -> Result<Route, String> {
    let mut pts: Vec<Coord> = Vec::with_capacity(waypoints.len() * 2 + 1);
    for &c in waypoints {
        if pts.last().map(|p| p.lon == c.lon && p.lat == c.lat) != Some(true) {
            pts.push(c);
        }
    }
    if pts.len() < 2 {
        return Err("自定义路径至少需要 2 个不重复的点".into());
    }
    match close {
        CloseMode::Closed => {
            let first = pts[0];
            let last = *pts.last().unwrap();
            if first.lon != last.lon || first.lat != last.lat {
                pts.push(first);
            }
        }
        CloseMode::RoundTrip => {
            // 去程 + 逆行回程；末点与回程首点重合，跳过以免零长段。
            let rev: Vec<Coord> = pts[..pts.len() - 1].iter().rev().copied().collect();
            pts.extend(rev);
        }
        CloseMode::OneWay => {}
    }
    let g = RoadGraph::from_polyline(&pts);
    let radius = radius_m.max(0.5);
    let points = smooth::smooth_and_sample(
        &g,
        &pts,
        radius,
        threshold_deg.clamp(0.0, 179.0),
        step_m.max(0.1),
    );
    if points.is_empty() {
        return Err("自定义路径平滑后为空（可能所有点重合）".into());
    }
    let length_m = points.last().map(|p| p.s).unwrap_or(0.0);
    Ok(Route { points, length_m })
}

/// 入口（软/硬点分离）：`waypoints[0]` 为起点；`must` 为强制必经点（按序，不含
/// 起点，可为空）；`waypoints[1..]` 为软引导点（仅用于方向锚点，不必全经过）。
pub fn plan_route_split(
    net: &RoadGraph,
    waypoints: &[Coord],
    must: &[Coord],
    target_len_m: f64,
    seed: u64,
    opts: &RouteOptions,
) -> Result<Route, String> {
    if waypoints.is_empty() {
        return Err("无起点".into());
    }
    let mut g = net.clone();

    // 合并查询点（起点 + 必经点 + 软引导点），交由投影函数按坐标去重。
    let mut queries = vec![waypoints[0]];
    queries.extend(must.iter().copied());
    queries.extend(waypoints[1..].iter().copied());
    let vnodes = project::add_virtual_nodes(&mut g, &queries)?;

    let start = vnodes[0];
    let must_nodes: Vec<NodeIndex> = vnodes[1..1 + must.len()]
        .iter()
        .copied()
        .filter(|n| *n != start)
        .collect();
    let anchor_nodes: Vec<NodeIndex> = vnodes[1 + must.len()..].to_vec();

    let path = route::plan_loop(
        &g,
        start,
        &must_nodes,
        &anchor_nodes,
        target_len_m,
        seed,
        opts,
    )?;
    let coords = route::path_coords(&g, &path);
    let points =
        smooth::smooth_and_sample(&g, &coords, opts.min_radius_m, opts.turn_threshold_deg, 1.0);
    if points.is_empty() {
        return Err("路线平滑后为空".into());
    }
    let length_m = points.last().map(|p| p.s).unwrap_or(0.0);
    Ok(Route { points, length_m })
}
