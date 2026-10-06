//! 地图画布绘制原语：等距投影 + 拖拽平移 + 滚轮缩放。
//!
//! 供「路网」页与「跑步」页复用，纯离线绘制折线/多边形/点。

use eframe::egui;
use egui::{Color32, Pos2, Rect, Stroke, Vec2};

use crate::track::geom::{MET_PER_DEG_LAT, MET_PER_DEG_LNG};

/// 地图视图状态（世界坐标为度）。
pub struct MapState {
    /// 参考中心（lat, lon）。
    pub center: (f64, f64),
    /// 屏幕每米像素（缩放）。
    pub scale: f32,
    /// 平移偏移（像素）。
    pub offset: Vec2,
    /// 最近是否被拖拽过（供外部判断）。
    pub dragged: bool,
}

impl Default for MapState {
    fn default() -> Self {
        MapState {
            center: (0.0, 0.0),
            scale: 0.3,
            offset: Vec2::ZERO,
            dragged: false,
        }
    }
}

impl MapState {
    /// 依据一组点（lat, lon）自动适配视野。
    pub fn fit(&mut self, rect: Rect, pts: &[(f64, f64)]) {
        if pts.len() < 2 {
            return;
        }
        let (mut slat, mut slon) = (0.0f64, 0.0f64);
        for (la, lo) in pts {
            slat += *la;
            slon += *lo;
        }
        let n = pts.len() as f64;
        self.center = (slat / n, slon / n);
        let (mut minx, mut maxx, mut miny, mut maxy) = (
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
        );
        for (la, lo) in pts {
            let x = (lo - self.center.1) * MET_PER_DEG_LNG;
            let y = (la - self.center.0) * MET_PER_DEG_LAT;
            minx = minx.min(x);
            maxx = maxx.max(x);
            miny = miny.min(y);
            maxy = maxy.max(y);
        }
        let w = (maxx - minx).max(40.0);
        let h = (maxy - miny).max(40.0);
        let sx = rect.width() as f64 / w;
        let sy = rect.height() as f64 / h;
        self.scale = ((sx.min(sy)) * 0.92) as f32;
        self.offset = Vec2::ZERO;
    }

    /// 度坐标 → 屏幕坐标。
    pub fn to_screen(&self, rect: Rect, lat: f64, lon: f64) -> Pos2 {
        let x = (lon - self.center.1) * MET_PER_DEG_LNG * self.scale as f64;
        let y = -(lat - self.center.0) * MET_PER_DEG_LAT * self.scale as f64;
        let c = rect.center();
        Pos2::new(
            (c.x as f64 + x + self.offset.x as f64) as f32,
            (c.y as f64 + y + self.offset.y as f64) as f32,
        )
    }

    /// 处理拖拽/缩放交互（需在分配好 rect 后调用）。
    pub fn interact(&mut self, ui: &egui::Ui, rect: Rect) {
        self.dragged = false;
        let (drag, scroll, hovered) = ui.input(|i| {
            // interact_pos 会考虑图层遮挡：指针位于文本框等上层控件时返回其位置，
            // 此时 rect.contains 为假，滚轮不会误触发地图缩放。
            let hover = i
                .pointer
                .interact_pos()
                .map(|p| rect.contains(p))
                .unwrap_or(false);
            let dragging = hover && i.pointer.primary_down() && i.pointer.is_decidedly_dragging();
            (dragging, i.raw_scroll_delta.y, hover)
        });
        if drag {
            let d = ui.input(|i| i.pointer.delta());
            self.offset += d;
            self.dragged = true;
        }
        // 仅在指针悬停于画布内时缩放，避免在经纬度等输入框滚动时连带缩放地图。
        if scroll != 0.0 && hovered {
            let factor = (scroll * 0.0015).exp();
            self.scale = (self.scale * factor).clamp(0.002, 200.0);
        }
    }

    /// 绘制折线。
    ///
    /// 屏幕空间抽稀：路径按米级间隔采样（可达上万点），缩小后相邻点会落在同一
    /// 像素附近，逐段描边的抗锯齿会产生细缝，视觉上「断断续续」。这里丢弃屏幕
    /// 距离 < `MIN_SEG_PX` 的相邻点（末点始终保留），既消除缝隙又减少顶点开销。
    pub fn draw_polyline(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        pts: &[(f64, f64)],
        color: Color32,
        width: f32,
    ) {
        if pts.len() < 2 {
            return;
        }
        /// 相邻屏幕点的最小间距（像素）；小于该值的点会产生描边缝隙。
        const MIN_SEG_PX: f32 = 0.75;

        let mut ps: Vec<Pos2> = Vec::with_capacity(pts.len());
        for (la, lo) in pts {
            let p = self.to_screen(rect, *la, *lo);
            if let Some(last) = ps.last() {
                if (p - *last).length() < MIN_SEG_PX {
                    continue;
                }
            }
            ps.push(p);
        }
        // 强制保留真实末点（抽稀可能把末点视作过近而丢弃，导致终点缺失）。
        if let Some(&(la, lo)) = pts.last() {
            let last = self.to_screen(rect, la, lo);
            match ps.last() {
                Some(q) if (*q - last).length() < MIN_SEG_PX => {
                    if let Some(slot) = ps.last_mut() {
                        *slot = last;
                    }
                }
                _ => ps.push(last),
            }
        }
        if ps.len() < 2 {
            return;
        }
        painter.add(egui::Shape::line(ps, Stroke::new(width, color)));
    }

    pub fn draw_polygon(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        pts: &[(f64, f64)],
        color: Color32,
        width: f32,
    ) {
        if pts.len() < 3 {
            return;
        }
        let mut ps: Vec<Pos2> = pts
            .iter()
            .map(|(la, lo)| self.to_screen(rect, *la, *lo))
            .collect();
        ps.push(ps[0]);
        painter.add(egui::Shape::closed_line(ps, Stroke::new(width, color)));
    }

    pub fn draw_point(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        lat: f64,
        lon: f64,
        color: Color32,
        radius: f32,
    ) {
        let c = self.to_screen(rect, lat, lon);
        painter.circle_filled(c, radius, color);
    }
}

/// 收集一组折线/环的所有点，用于 fit。
pub fn collect_bounds(edges: &[Vec<(f64, f64)>]) -> Vec<(f64, f64)> {
    let mut pts: Vec<(f64, f64)> = Vec::new();
    for e in edges.iter().take(400) {
        for p in e {
            pts.push(*p);
        }
    }
    pts
}

/// 地图图例：颜色方块 + 说明文字。
pub fn legend(ui: &mut egui::Ui, items: &[(&str, Color32)]) {
    ui.horizontal(|ui| {
        ui.label("图例：");
        for (label, color) in items {
            let (rect, _) = ui.allocate_exact_size(Vec2::new(10.0, 10.0), egui::Sense::hover());
            ui.painter()
                .rect_filled(rect, egui::Rounding::same(2.0), *color);
            ui.label(*label);
        }
    });
}
