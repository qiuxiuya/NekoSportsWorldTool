//! 跑步页：恒 sportType=1；距离/配速范围 + 开始时间（随机或指定，最多前 3 天）。
//!
//! 开始时间：「日期」下拉（今天 / 1-3 天前）两种模式共用；随机模式在该日 7:00-20:00
//! 内抽样，并保证「开始 + 用时」不越过当前时刻；指定模式完全按用户填写的时/分
//! （尚未到达时按当前时刻），「换一版」只重掷运动量，不动时刻。

use super::map::MapState;
use super::{mobile, theme, App};
use crate::track::custom::Datum;
use crate::track::generate_road::{PathClose, RouteMode};
use chrono::{Datelike, Duration, Local, TimeZone, Timelike};
use eframe::egui;

/// 随机时刻窗口（含端点）：7:00-20:00。
const RAND_HOUR_LO: u32 = 7;
const RAND_HOUR_HI: u32 = 20;
/// flow.rs 提交时会给开始时间加 0-4s 抖动，随机模式抽样时预留，避免「开始 + 用时」越过当前时刻。
const JITTER_MARGIN_MS: i64 = 5_000;

/// 指定日期（days_ago 天前）的 7:00-20:00 内均匀抽样一个时刻，且不晚于 latest_ms。
///
/// latest_ms = 当前时刻 - 用时 - 抖动余量。若该日窗口整体晚于 latest_ms（例如凌晨选「今天」），
/// 退化为 latest_ms，宁可贴近当前时间，也不产生「未来开跑」。
fn random_time_ago(days_ago: i64, latest_ms: i64) -> i64 {
    let base = Local::now() - Duration::days(days_ago.clamp(0, 3));
    let lo = Local
        .with_ymd_and_hms(base.year(), base.month(), base.day(), RAND_HOUR_LO, 0, 0)
        .single();
    let hi = Local
        .with_ymd_and_hms(base.year(), base.month(), base.day(), RAND_HOUR_HI, 0, 0)
        .single();
    let (lo_ms, hi_ms) = match (lo, hi) {
        (Some(l), Some(h)) => (l.timestamp_millis(), h.timestamp_millis()),
        _ => return latest_ms,
    };
    let hi_ms = hi_ms.min(latest_ms);
    let lo_ms = lo_ms.min(hi_ms);
    let span = (hi_ms - lo_ms).max(0) as f64;
    lo_ms + (rand::random::<f64>() * span) as i64
}

/// 指定时刻：days_ago(0=今天) + 时/分；超出 3 天或在未来时做钳制。
fn specified_time(days_ago: i64, hour: i64, minute: i64) -> i64 {
    let now = Local::now();
    let base = now - Duration::days(days_ago.clamp(0, 3));
    let h = hour.clamp(0, 23);
    let m = minute.clamp(0, 59);
    let t = Local
        .with_ymd_and_hms(base.year(), base.month(), base.day(), h as u32, m as u32, 0)
        .single()
        .map(|x| x.timestamp_millis())
        .unwrap_or_else(crate::crypto::envelope::now_ms);
    t.min(now.timestamp_millis())
}

/// 日期下拉的显示文本。
fn days_ago_label(days_ago: i64) -> String {
    match days_ago {
        0 => "今天".into(),
        1 => "昨天".into(),
        n => format!("{n} 天前"),
    }
}

#[derive(Default)]
pub struct RunPage {
    pub dist_min: f32,
    pub dist_max: f32,
    pub pace_min: f32,
    pub pace_max: f32,
    /// GPS 漂移距离（米）：相关漂移稳态幅度，越大轨迹越"松"。
    pub gps_drift_m: f32,
    /// 是否手动覆盖海拔（关闭时使用生成器海拔曲线）。
    pub manual_altitude_on: bool,
    /// 手动海拔下限（米，非负）；与上限相同视为固定单值。
    pub manual_altitude_min: f32,
    /// 手动海拔上限（米，非负）。
    pub manual_altitude_max: f32,
    /// 0=随机时刻 1=指定时刻
    pub start_mode: usize,
    pub days_ago: i64,
    pub hour: i64,
    pub minute: i64,
    pub face_check: bool,
    /// 预计算方案：参数变更时重抽样，提交直接使用
    pub plan: Option<RunPlan>,
    /// 路线算法模式
    pub route_mode: RouteMode,
    /// 地图视图状态
    pub map: MapState,
    /// 路网/自定义路径预览
    pub preview: Option<crate::track::generate_road::RoadPlan>,
    /// 预览是否过期（参数变更后置真）
    pub preview_stale: bool,
    /// 地图视野是否已适配
    pub map_fitted: bool,

    // ── 自定义路径 ─────────────────────────────────────────────
    /// 导入文件路径（仅作提示/回填）。
    pub custom_path: String,
    /// 原始路径文本（GPX/GeoJSON/纯文本，手输或导入）。
    pub custom_text: String,
    /// 导入坐标基准。
    pub custom_datum: Datum,
    /// 是否用已导入 OSM 路网建筑做 GPS 漂移放大。
    pub custom_use_buildings: bool,
    /// 文本解析提示（成功显示格式与点数，失败显示原因）。
    pub custom_msg: String,
    /// 解析后的 BD-09 折线缓存（None 表示未解析/解析失败）。
    pub custom_points_bd: Option<Vec<(f64, f64)>>,
    /// 文本是否被修改（需重新解析）。
    pub custom_dirty: bool,
    /// 上次解析时的文本快照（用于检测 TextEdit 直接改动）。
    pub custom_text_seen: String,
    /// 上一帧的路线模式（用于检测模式切换并让预览失效）。
    pub last_route_mode: RouteMode,
    /// 首尾走法（自定义 / 高德共用）。
    pub custom_close: PathClose,
    /// 高德 Web 服务 Key。
    pub amap_key: String,
    /// 高德安全密钥 securityJsCode。
    pub amap_jscode: String,
    /// 高德步行规划结果（BD-09 折线），预览与提交共用。
    pub amap_points_bd: Option<Vec<(f64, f64)>>,
    /// 高德规划是否进行中。
    pub amap_busy: bool,
    /// 高德规划状态文本。
    pub amap_msg: String,
    /// 本帧是否请求启动高德规划（由 App 负责起后台线程）。
    pub amap_plan_requested: bool,
}

/// 一次提交的确定方案（进入页面/参数变更时抽样生成）。
#[derive(Debug, Clone)]
pub struct RunPlan {
    pub dist_min: f32,
    pub dist_max: f32,
    pub pace_min: f32,
    pub pace_max: f32,
    pub start_mode: usize,
    pub days_ago: i64,
    pub hour: i64,
    pub minute: i64,
    /// 公里
    pub dist: f64,
    /// 秒/km
    pub pace: f32,
    /// 秒
    pub dur: i64,
    pub start_ms: i64,
    /// 本次方案的随机种子（预览与提交共用，保证所见即所得）
    pub seed: u64,
}

impl RunPage {
    /// 重新解析自定义路径文本（按当前基准），并落盘。
    ///
    /// 失败时清空 `custom_points_bd` 并记录原因，界面据此提示；成功时缓存 BD 折线。
    pub fn reparse_custom(&mut self) {
        let text = self.custom_text.clone();
        self.preview_stale = true;
        // 高德模式下文本框内容是本应用写入的 BD-09，始终按 BD-09 解析，避免用户切换
        // 显示基准后解析结果漂移（几何已由 active_points_bd 的转换为 BD-09 保持一致）。
        let parse_datum = if self.route_mode == RouteMode::Amap {
            crate::track::custom::Datum::Bd09
        } else {
            self.custom_datum
        };
        // 路径点变化后旧的沿路规划结果失效，需重新点「规划道路」。
        self.amap_points_bd = None;
        self.amap_msg.clear();
        if text.trim().is_empty() {
            self.custom_points_bd = None;
            self.custom_msg = "（未填写路径）".into();
            self.custom_dirty = false;
            self.custom_text_seen = text.clone();
            let _ = crate::api::model::save_custom_route(&text);
            return;
        }
        match crate::track::custom::parse_route(&text, parse_datum) {
            Ok(parsed) => {
                self.custom_msg = format!("√ {} · {} 个点", parsed.format, parsed.points_bd.len());
                self.custom_points_bd = Some(parsed.points_bd);
            }
            Err(e) => {
                self.custom_msg = format!("⚠ {e}");
                self.custom_points_bd = None;
            }
        }
        self.custom_dirty = false;
        self.custom_text_seen = text.clone();
        // 持久化原始文本；失败不阻断界面。
        let _ = crate::api::model::save_custom_route(&text);
    }

    /// 检测 TextEdit 对文本的直接改动（含粘贴）：变化即标记为待重解析。
    pub fn sync_custom_edit(&mut self) {
        if self.custom_text != self.custom_text_seen {
            self.custom_dirty = true;
        }
    }

    /// 若文本有改动则重新解析（供逐帧调用）。
    pub fn ensure_custom_parsed(&mut self) {
        if self.custom_dirty {
            self.reparse_custom();
        }
    }

    /// 自定义路径是否已解析出可用折线。
    pub fn custom_ready(&self) -> bool {
        self.custom_points_bd
            .as_ref()
            .map(|p| p.len() >= 2)
            .unwrap_or(false)
    }

    /// 高德规划是否已产出可用折线。
    pub fn amap_ready(&self) -> bool {
        self.amap_points_bd
            .as_ref()
            .map(|p| p.len() >= 2)
            .unwrap_or(false)
    }

    /// 当前模式下的几何折线（自定义 / 高德），供预览与提交共用。
    pub fn active_points_bd(&self) -> Option<Vec<(f64, f64)>> {
        match self.route_mode {
            RouteMode::Amap => self.amap_points_bd.clone(),
            _ => self.custom_points_bd.clone(),
        }
    }

    /// 当前模式是否已具备可用路径。
    pub fn route_ready(&self) -> bool {
        match self.route_mode {
            RouteMode::Amap => self.amap_ready(),
            RouteMode::Custom => self.custom_ready(),
            _ => true,
        }
    }

    /// 当前模式下的阻断提示（不可用时展示）。
    pub fn route_block_msg(&self) -> String {
        if self.route_mode == RouteMode::Amap {
            if self.amap_msg.is_empty() {
                "请先填「高德路径」点并点击「规划道路」".into()
            } else {
                self.amap_msg.clone()
            }
        } else if self.custom_msg.is_empty() {
            "请填写或导入至少 2 个点".into()
        } else {
            self.custom_msg.clone()
        }
    }

    /// 拼装提交用折线路径（坐标 + 走法 + 可选建筑 SDF 数据）。
    pub fn custom_route_payload(&self, osm_path: &str) -> crate::api::flow::CustomRoute {
        let buildings_bd = if self.custom_use_buildings {
            crate::track::generate_road::load_buildings_bd(osm_path)
        } else {
            Vec::new()
        };
        crate::api::flow::CustomRoute {
            points_bd: self.active_points_bd().unwrap_or_default(),
            buildings_bd,
            close: self.custom_close,
        }
    }

    /// 高德步行规划用的点序列（GCJ-02，按走法拼接）。
    ///
    /// 输入为 BD-09 路径点：先转回 GCJ-02 再交给高德；闭环追加首点、往返追加
    /// 逆序回程、单程保持原序。
    pub fn amap_sequence_gcj(&self) -> Result<Vec<(f64, f64)>, String> {
        let pts_bd = self
            .custom_points_bd
            .clone()
            .ok_or("请先填写或导入至少 2 个路径点")?;
        if pts_bd.len() < 2 {
            return Err("至少需要 2 个路径点".into());
        }
        let mut seq: Vec<(f64, f64)> = pts_bd
            .iter()
            .map(|&(la, lo)| crate::track::wire::bd09_to_gcj02(la, lo))
            .collect();
        match self.custom_close {
            PathClose::Closed => {
                let first = seq[0];
                let last = *seq.last().unwrap();
                if (first.0 - last.0).abs() > 1e-9 || (first.1 - last.1).abs() > 1e-9 {
                    seq.push(first);
                }
            }
            PathClose::RoundTrip => {
                let mut back: Vec<(f64, f64)> = seq[..seq.len() - 1].iter().rev().copied().collect();
                seq.append(&mut back);
            }
            PathClose::OneWay => {}
        }
        Ok(seq)
    }

    /// 距离/配速是否与方案一致（不一致需重掷运动参数）。
    fn shape_matches(&self, p: &RunPlan) -> bool {
        p.dist_min == self.dist_min
            && p.dist_max == self.dist_max
            && p.pace_min == self.pace_min
            && p.pace_max == self.pace_max
    }

    /// 开始时间控件是否与方案一致。
    ///
    /// 随机模式的时刻是抽样结果（不是输入），只比对日期；指定模式的时/分是用户输入，需回比。
    fn time_matches(&self, p: &RunPlan) -> bool {
        if p.start_mode != self.start_mode || p.days_ago != self.days_ago {
            return false;
        }
        self.start_mode == 0 || (p.hour == self.hour && p.minute == self.minute)
    }

    /// 开跑时刻上限：当前时刻 - 用时 - 抖动余量（flow.rs 提交时还会再加 0-4s）。
    fn latest_start_ms(dur: i64) -> i64 {
        crate::crypto::envelope::now_ms() - dur * 1000 - JITTER_MARGIN_MS
    }

    /// 按当前参数抽样运动量（距离 / 配速 / 用时）。
    fn sample_shape(&self) -> (f64, f32, i64) {
        let (lo, hi) = (
            self.dist_min.min(self.dist_max),
            self.dist_min.max(self.dist_max),
        );
        let (plo, phi) = (
            self.pace_min.min(self.pace_max),
            self.pace_min.max(self.pace_max),
        );
        let pace = plo + (phi - plo) * rand::random::<f32>();
        let dist = (lo + (hi - lo) * rand::random::<f32>()) as f64;
        let dur = (dist * pace as f64).round() as i64;
        (dist, pace, dur)
    }

    /// 按当前模式与日期取一个开始时刻（随机模式抽样，指定模式采纳输入框）。
    fn pick_time(&self) -> i64 {
        if self.start_mode == 0 {
            random_time_ago(self.days_ago, Self::latest_start_ms(self.dur_hint()))
        } else {
            specified_time(self.days_ago, self.hour, self.minute)
        }
    }

    /// 用于随机抽样的时长上限：已抽样的方案优先，否则按当前配速区间上界估一个。
    fn dur_hint(&self) -> i64 {
        match &self.plan {
            Some(p) => p.dur,
            None => {
                let hi = self.dist_min.max(self.dist_max) as f64;
                let phi = self.pace_min.max(self.pace_max) as f64;
                (hi * phi).round() as i64
            }
        }
    }

    /// 运动参数变更时重抽距离/配速/用时，保留已选开始时刻（指定模式的时刻是用户输入，不动）。
    fn regen_shape(&mut self) {
        let (dist, pace, dur) = self.sample_shape();
        let start_ms = match (&self.plan, self.start_mode) {
            // 随机模式沿用已抽样的时刻（仅按新用时收紧上限），避免拖动距离时时刻乱跳
            (Some(p), 0) => p.start_ms.min(Self::latest_start_ms(dur)),
            _ => self.pick_time(),
        };
        self.write_plan(dist, pace, dur, start_ms);
    }

    /// 只重算开始时刻，保留已抽样的运动量（日期 / 模式变更时用）。
    fn resync_time(&mut self) {
        let start_ms = self.pick_time();
        if let Some(p) = self.plan.as_mut() {
            p.start_mode = self.start_mode;
            p.days_ago = self.days_ago;
            p.hour = self.hour;
            p.minute = self.minute;
            p.start_ms = start_ms;
        }
    }

    /// 把当前参数与已算好的量写成方案。
    fn write_plan(&mut self, dist: f64, pace: f32, dur: i64, start_ms: i64) {
        self.plan = Some(RunPlan {
            dist_min: self.dist_min,
            dist_max: self.dist_max,
            pace_min: self.pace_min,
            pace_max: self.pace_max,
            start_mode: self.start_mode,
            days_ago: self.days_ago,
            hour: self.hour,
            minute: self.minute,
            dist,
            pace,
            dur,
            start_ms,
            seed: rand::random::<u64>(),
        });
        self.preview_stale = true;
    }

    /// 「换一版」：重掷运动量；随机模式下同时换一个开始时刻。
    ///
    /// 指定模式的时刻由用户填写，按钮不碰它（用户填什么就是什么）。
    pub fn regen_plan(&mut self) {
        let (dist, pace, dur) = self.sample_shape();
        let start_ms = if self.start_mode == 0 {
            random_time_ago(self.days_ago, Self::latest_start_ms(dur))
        } else {
            specified_time(self.days_ago, self.hour, self.minute)
        };
        self.write_plan(dist, pace, dur, start_ms);
    }

    /// 确保方案与当前参数一致（参数变更补齐 / 提交前兜底）。
    ///
    /// 两个一致性判定都在改写方案「之前」求值：否则 regen_shape 会把当前参数写进方案，
    /// 同帧内「距离 + 日期」一起改时日期变化将被漏掉。
    pub fn ensure_plan(&mut self) {
        let shape_ok = self.plan.as_ref().map(|p| self.shape_matches(p)) == Some(true);
        let time_ok = self.plan.as_ref().map(|p| self.time_matches(p)) == Some(true);
        if !shape_ok {
            self.regen_shape();
        }
        if !time_ok {
            self.resync_time();
        }
    }

    /// 随机模式的抽样结果是否落在所选日期的 7:00-20:00 窗口内。
    ///
    /// 凌晨选「今天」时窗口尚未到达，`random_time_ago` 会退化到贴近当前时刻，
    /// 此时界面需要说明，避免用户以为刻意选了凌晨。
    fn random_window_ok(&self) -> bool {
        let Some(p) = &self.plan else { return true };
        let Some(t) = Local.timestamp_millis_opt(p.start_ms).single() else {
            return true;
        };
        let day = (Local::now() - Duration::days(self.days_ago.clamp(0, 3))).date_naive();
        t.date_naive() == day && (RAND_HOUR_LO..=RAND_HOUR_HI).contains(&t.hour())
    }
}

impl App {
    pub fn draw_run(&mut self, ui: &mut egui::Ui) {
        egui::ScrollArea::vertical()
            .auto_shrink([false, false])
            .show(ui, |ui| {
                self.draw_run_content(ui);
            });
    }

    fn draw_run_content(&mut self, ui: &mut egui::Ui) {
        // 与上一帧比较（而非本次赋值），才能检测出用户切换了路线模式。
        let mode_changed = self.run_page.route_mode != self.run_page.last_route_mode;
        if mode_changed {
            self.run_page.last_route_mode = self.run_page.route_mode;
            self.run_page.preview_stale = true;
            self.run_page.map_fitted = false;
        }
        self.poll_amap_plan();
        {
            let page = &mut self.run_page;
            let compact = mobile::compact_ui(ui);
            let mut draw_distance_inputs = |ui: &mut egui::Ui| {
                mobile::drag_f32(
                    ui,
                    "run_dist_min",
                    &mut page.dist_min,
                    0.5..=20.0,
                    0.05,
                    2,
                    " km",
                );
                ui.label("至");
                mobile::drag_f32(
                    ui,
                    "run_dist_max",
                    &mut page.dist_max,
                    0.5..=20.0,
                    0.05,
                    2,
                    " km",
                );
            };
            if compact {
                ui.label("距离范围（km）：");
                ui.horizontal(draw_distance_inputs);
            } else {
                ui.horizontal(|ui| {
                    ui.label("距离范围（km）：");
                    draw_distance_inputs(ui);
                });
            }
            let mut draw_pace_inputs = |ui: &mut egui::Ui| {
                mobile::drag_f32(
                    ui,
                    "run_pace_min",
                    &mut page.pace_min,
                    180.0..=520.0,
                    5.0,
                    0,
                    "",
                );
                ui.label("至");
                mobile::drag_f32(
                    ui,
                    "run_pace_max",
                    &mut page.pace_max,
                    180.0..=520.0,
                    5.0,
                    0,
                    "",
                );
            };
            if compact {
                ui.label("配速范围（秒/km）：");
                ui.horizontal(draw_pace_inputs);
            } else {
                ui.horizontal(|ui| {
                    ui.label("配速范围（秒/km）：");
                    draw_pace_inputs(ui);
                });
            }
            mobile::row(ui, |ui| {
                ui.label("GPS 漂移距离（米）：");
                mobile::drag_f32(
                    ui,
                    "run_gps_drift",
                    &mut page.gps_drift_m,
                    0.0..=8.0,
                    0.1,
                    1,
                    "",
                );
                ui.label("越大轨迹越松，0=贴合道路");
            });
            mobile::row(ui, |ui| {
                ui.label("手动海拔（米）：");
                ui.checkbox(&mut page.manual_altitude_on, "启用");
                mobile::drag_f32(
                    ui,
                    "run_alt_min",
                    &mut page.manual_altitude_min,
                    -500.0..=9000.0,
                    0.5,
                    1,
                    "",
                );
                ui.label("至");
                mobile::drag_f32(
                    ui,
                    "run_alt_max",
                    &mut page.manual_altitude_max,
                    -500.0..=9000.0,
                    0.5,
                    1,
                    "",
                );
                ui.label("两框相同=固定海拔；取消勾选=自动");
            });
            mobile::row(ui, |ui| {
                ui.label("开始时间：");
                ui.radio_value(&mut page.start_mode, 0, "随机时刻");
                ui.radio_value(&mut page.start_mode, 1, "指定时刻");
                ui.label("日期：");
                egui::ComboBox::from_id_salt("run_days_ago")
                    .width(96.0)
                    .selected_text(days_ago_label(page.days_ago))
                    .show_ui(ui, |ui| {
                        for d in 0..=3 {
                            ui.selectable_value(&mut page.days_ago, d, days_ago_label(d));
                        }
                    });
                if page.start_mode == 0 {
                    ui.label(format!("（{RAND_HOUR_LO}:00-{RAND_HOUR_HI}:00 内随机）"));
                } else {
                    ui.label("时刻：");
                    mobile::drag_i64(ui, "run_hour", &mut page.hour, 0..=23, 1.0, "", " 点");
                    ui.label(":");
                    mobile::drag_i64(ui, "run_minute", &mut page.minute, 0..=59, 1.0, "", "");
                }
            });
            if page.start_mode == 1 && page.days_ago == 0 {
                // 今天 + 指定时刻：提示是否落在未来
                let now = Local::now();
                let spec = Local
                    .with_ymd_and_hms(
                        now.year(),
                        now.month(),
                        now.day(),
                        page.hour as u32,
                        page.minute as u32,
                        0,
                    )
                    .single();
                if let Some(t) = spec {
                    if t.timestamp_millis() > crate::crypto::envelope::now_ms() {
                        ui.colored_label(
                            theme::warn(),
                            "指定时刻在今天且尚未到达，将按当前时间提交",
                        );
                    }
                }
            }
            mobile::row(ui, |ui| {
                ui.label("人脸校验标记：");
                ui.checkbox(&mut page.face_check, "faceCheck=1");
            });
            mobile::row(ui, |ui| {
                ui.label("路线算法：");
                ui.selectable_value(&mut page.route_mode, RouteMode::Legacy, "经典打卡点环");
                ui.selectable_value(&mut page.route_mode, RouteMode::Road, "真实道路路由");
                ui.selectable_value(&mut page.route_mode, RouteMode::Custom, "自定义路径");
                ui.selectable_value(&mut page.route_mode, RouteMode::Amap, "高德路径");
            });
            if page.route_mode == RouteMode::Road && self.network.is_none() {
                ui.colored_label(theme::warn(), "真实道路路由需先在「路网」页导入 OSM");
            }
            if page.route_mode.is_polyline_based() {
                Self::draw_custom_editor(ui, page, compact);
            }
        }

        // 切换到真实道路路由时按需拉取电子围栏与实时点位（经典模式不触发这些端点）。
        if mode_changed && self.run_page.route_mode == RouteMode::Road {
            self.refresh_fence();
            self.refresh_points();
        }
        // 切换到高德路径时拉取实时点位（检查点）作为路径点来源。
        if mode_changed && self.run_page.route_mode == RouteMode::Amap {
            self.refresh_points();
        }

        ui.add_space(8.0);
        let fmt_pace = |s: f32| format!("{}:{:02}", (s / 60.0) as i64, (s as i64) % 60);
        let fmt_dur = |s: i64| {
            if s >= 3600 {
                format!("{}:{:02}:{:02}", s / 3600, s % 3600 / 60, s % 60)
            } else {
                format!("{}:{:02}", s / 60, s % 60)
            }
        };
        // 参数变更时补方案；显示本次提交的确定方案
        self.run_page.ensure_plan();
        self.draw_run_warnings(ui);
        let plan_label = match &self.run_page.plan {
            Some(p) => {
                let start = chrono::Local
                    .timestamp_millis_opt(p.start_ms)
                    .single()
                    .map(|t| t.format("%Y-%m-%d %H:%M").to_string())
                    .unwrap_or_default();
                let (dist, pace, dur) = (p.dist, p.pace, p.dur);
                format!(
                    "本次方案：距离 {dist:.2} km · 配速 {}/km · 用时 {} · 开始 {start}",
                    fmt_pace(pace),
                    fmt_dur(dur)
                )
            }
            None => String::new(),
        };
        mobile::row(ui, |ui| {
            ui.colored_label(theme::plain(), plan_label);
            if ui.small_button("换一版").clicked() {
                self.run_page.regen_plan();
            }
        });

        self.draw_run_map(ui);

        ui.add_space(8.0);
        let enabled = !self.run_busy && self.session.is_some();
        let btn = if self.run_busy {
            theme::primary_btn("提交中…")
        } else {
            theme::primary_btn("开始跑步")
        };
        mobile::row(ui, |ui| {
            if ui.add_enabled(enabled, btn).clicked() {
                self.start_run();
            }
            if self.session.is_none() {
                ui.label("（请先登录）");
            }
        });
    }

    /// 自定义 / 高德路径编辑区：走法、导入文件、手输经纬度、坐标基准、解析状态。
    ///
    /// 文本是唯一数据源，`custom_path` 仅用于导入时读文件；变更后置 `custom_dirty`，
    /// 由 `ensure_custom_parsed` 在编辑区末尾统一解析一次。
    fn draw_custom_editor(ui: &mut egui::Ui, page: &mut RunPage, compact: bool) {
        let amap = page.route_mode == RouteMode::Amap;
        ui.add_space(4.0);
        ui.label(if amap {
            "路径点（默认自动获取跑步检查点；也可手动粘贴「纬度,经度」或 GPX/GeoJSON）："
        } else {
            "自定义路径（GPX / GeoJSON / 文本「纬度,经度」）："
        });
        mobile::row(ui, |ui| {
            ui.label("走法：");
            ui.selectable_value(&mut page.custom_close, PathClose::Closed, "循环（首尾相连）");
            ui.selectable_value(&mut page.custom_close, PathClose::RoundTrip, "往返（原路返回）");
            ui.selectable_value(&mut page.custom_close, PathClose::OneWay, "单程");
        });
        mobile::row(ui, |ui| {
            ui.label("坐标基准：");
            let before = page.custom_datum;
            ui.selectable_value(&mut page.custom_datum, Datum::Wgs84, "WGS84（GPS）");
            ui.selectable_value(&mut page.custom_datum, Datum::Gcj02, "GCJ-02（高德）");
            ui.selectable_value(&mut page.custom_datum, Datum::Bd09, "BD-09（百度）");
            if page.custom_datum != before {
                // 高德路径：点击基准即把文本框坐标换算到新基准（保持地理位置不变），
                // 避免仅换解析基准导致整条路线漂移；GPX/GeoJSON 无法原地换算则退回重新解析。
                let amap_ready = page.amap_points_bd.as_ref().is_some_and(|p| p.len() >= 2);
                if amap
                    && amap_ready
                    && page.custom_points_bd.as_ref().is_some_and(|p| p.len() >= 2)
                    && !page.custom_text.trim().is_empty()
                {
                    if let Some(converted) = crate::track::custom::convert_text_datum(
                        &page.custom_text,
                        before,
                        page.custom_datum,
                    ) {
                        page.custom_text = converted;
                        page.custom_text_seen = page.custom_text.clone();
                        page.custom_dirty = true;
                        page.custom_msg = format!(
                            "√ 坐标基准 {} → {} 已自动换算",
                            before.as_str(),
                            page.custom_datum.as_str()
                        );
                    } else {
                        page.custom_dirty = true;
                    }
                } else {
                    page.custom_dirty = true;
                }
            }
            let mut use_bld = page.custom_use_buildings;
            ui.checkbox(&mut use_bld, "用路网建筑模拟 GPS 漂移");
            page.custom_use_buildings = use_bld;
        });

        // 文件导入：桌面「浏览…」选文件；两端均可用路径框 + 加载。
        let mut load_from_path = false;
        mobile::row(ui, |ui| {
            ui.label("文件：");
            mobile::text_edit(
                ui,
                "run_custom_path",
                &mut page.custom_path,
                crate::platform::InputKind::Text,
                if compact { 200.0 } else { 320.0 },
            );
            #[cfg(not(target_os = "android"))]
            if ui.button("浏览…").clicked() {
                if let Some(p) = rfd::FileDialog::new()
                    .add_filter("轨迹", &["gpx", "geojson", "json", "txt", "csv"])
                    .pick_file()
                {
                    page.custom_path = p.display().to_string();
                    load_from_path = true;
                }
            }
            if ui.button("加载").clicked() {
                load_from_path = true;
            }
        });

        // 拖拽导入（桌面）
        #[cfg(not(target_os = "android"))]
        {
            let dropped: Vec<String> = ui.ctx().input(|i| {
                i.raw
                    .dropped_files
                    .iter()
                    .filter_map(|f| f.path.as_ref().map(|p| p.display().to_string()))
                    .collect()
            });
            if let Some(p) = dropped.into_iter().find(|p| !p.is_empty()) {
                page.custom_path = p;
                load_from_path = true;
            }
        }

        // 文本输入（桌面多行且限高滚动；Android 单行，用 ; / | 分隔多点）
        // 限高 140px：粘贴整份 GPX/GeoJSON 时编辑框不会撑满整屏。
        mobile::text_edit_multiline(ui, "run_custom_text", &mut page.custom_text, 5, 140.0);
        if !page.custom_path.is_empty() && page.custom_text.is_empty() {
            ui.colored_label(theme::plain(), "（可从上方文件加载，或直接粘贴内容）");
        }
        // 展示规模：长文本时给出点数与字符数，便于确认已解析。
        let text_lines = page.custom_text.lines().filter(|l| !l.trim().is_empty()).count();
        if text_lines > 6 {
            ui.colored_label(
                theme::plain(),
                format!(
                    "（已输入 {} 行 / {} 字符，编辑框内可滚动查看）",
                    text_lines,
                    page.custom_text.chars().count()
                ),
            );
        }

        if load_from_path && !page.custom_path.is_empty() {
            match std::fs::read_to_string(&page.custom_path) {
                Ok(content) => {
                    page.custom_text = content;
                    page.custom_dirty = true;
                }
                Err(e) => page.custom_msg = format!("⚠ 读取文件失败: {e}"),
            }
        }

        // 文本被外部改动（含粘贴/加载）后需要重新解析：比对快照。
        page.sync_custom_edit();
        page.ensure_custom_parsed();

        if amap {
            mobile::row(ui, |ui| {
                ui.label("高德 Key：");
                mobile::text_edit(
                    ui,
                    "run_amap_key",
                    &mut page.amap_key,
                    crate::platform::InputKind::Text,
                    if compact { 200.0 } else { 300.0 },
                );
            });
            mobile::row(ui, |ui| {
                ui.label("安全密钥：");
                mobile::text_edit(
                    ui,
                    "run_amap_jscode",
                    &mut page.amap_jscode,
                    crate::platform::InputKind::Text,
                    if compact { 200.0 } else { 300.0 },
                );
                ui.label("（Key 启用安全密钥时填 securityJsCode）");
            });
        }

        mobile::row(ui, |ui| {
            if page.custom_ready() {
                ui.colored_label(theme::ok(), &page.custom_msg);
            } else {
                ui.colored_label(theme::warn(), &page.custom_msg);
            }
            if ui.button("重新解析").clicked() {
                page.reparse_custom();
            }
            if ui.button("清空").clicked() {
                page.custom_text.clear();
                page.custom_path.clear();
                page.custom_points_bd = None;
                page.custom_msg = "（未填写路径）".into();
                page.custom_dirty = false;
                page.amap_points_bd = None;
                page.amap_msg.clear();
                let _ = crate::api::model::save_custom_route("");
            }
        });

        if amap {
            mobile::row(ui, |ui| {
                // 检查点来源提示：有缓存则显示数量，否则提示等待拉取。
                let n_cp = page.custom_points_bd.as_ref().map(|p| p.len()).unwrap_or(0);
                if n_cp >= 2 {
                    ui.colored_label(theme::ok(), format!("检查点 {n_cp} 个"));
                } else {
                    ui.colored_label(theme::warn(), "等待检查点…");
                }
                let can_plan = !page.amap_busy && !page.amap_key.trim().is_empty();
                let btn = if page.amap_busy {
                    theme::primary_btn("规划中…")
                } else {
                    theme::primary_btn("获取检查点并规划")
                };
                if ui.add_enabled(can_plan, btn).clicked() {
                    page.amap_plan_requested = true;
                    page.amap_msg.clear();
                }
                if page.amap_ready() {
                    let n = page.amap_points_bd.as_ref().map(|p| p.len()).unwrap_or(0);
                    ui.colored_label(
                        theme::ok(),
                        format!("√ 已沿道路规划 {n} 个点 · {}", page.custom_close.label()),
                    );
                } else if !page.amap_msg.is_empty() {
                    ui.colored_label(theme::warn(), &page.amap_msg);
                } else if page.amap_key.trim().is_empty() {
                    ui.colored_label(theme::warn(), "请填写高德 Key");
                } else {
                    ui.label("（点按钮自动获取检查点并沿真实道路规划）");
                }
            });
        }
    }

    /// 开始时间相关的提示。须在 `ensure_plan()` 之后调用，才对得上本次提交时刻。
    fn draw_run_warnings(&self, ui: &mut egui::Ui) {
        let page = &self.run_page;
        if page.start_mode == 0 {
            if !page.random_window_ok() {
                ui.colored_label(
                    theme::warn(),
                    format!(
                        "所选日期的 {RAND_HOUR_LO}:00-{RAND_HOUR_HI}:00 尚未到达，本次只能贴近当前时刻",
                    ),
                );
            }
            return;
        }
        // 今天 + 指定时刻：提示是否落在未来（与原有行为一致）
        if page.days_ago != 0 {
            return;
        }
        let now = Local::now();
        if let Some(t) = Local
            .with_ymd_and_hms(
                now.year(),
                now.month(),
                now.day(),
                page.hour.clamp(0, 23) as u32,
                page.minute.clamp(0, 59) as u32,
                0,
            )
            .single()
        {
            if t.timestamp_millis() > crate::crypto::envelope::now_ms() {
                ui.colored_label(theme::warn(), "指定时刻在今天且尚未到达，将按当前时间提交");
            }
        }
    }

    /// 轮询高德规划结果 / 触发新的规划请求（每帧调用）。
    fn poll_amap_plan(&mut self) {
        while let Ok(res) = self.amap_rx.try_recv() {
            self.run_page.amap_busy = false;
            let (result, logs) = match res {
                Ok((points, len, logs)) => (Ok((points, len)), logs),
                Err((e, logs)) => (Err(e), logs),
            };
            for line in logs {
                self.log.push(&line);
            }
            match result {
                Ok((points, len)) => {
                    // 规划完成后把全部沿路点自动填入路径框，便于查看/复用；
                    // 同步 text_seen 且清 dirty，避免下一帧 ensure_custom_parsed 重新解析把规划结果清掉。
                    let text = points
                        .iter()
                        .map(|(la, lo)| format!("{la:.6},{lo:.6}"))
                        .collect::<Vec<_>>()
                        .join("\n");
                    self.run_page.custom_datum = crate::track::custom::Datum::Bd09;
                    self.run_page.custom_points_bd = Some(points.clone());
                    self.run_page.custom_text = text;
                    self.run_page.custom_text_seen = self.run_page.custom_text.clone();
                    self.run_page.custom_dirty = false;
                    self.run_page.custom_msg =
                        format!("√ 已沿道路规划 {} 个点（已自动填入）", points.len());
                    self.run_page.amap_points_bd = Some(points);
                    self.run_page.amap_msg = format!("沿道路规划完成：约 {:.0} m", len);
                    self.run_page.preview_stale = true;
                    self.run_page.map_fitted = false;
                    self.status = self.run_page.amap_msg.clone();
                }
                Err(e) => {
                    self.run_page.amap_points_bd = None;
                    self.run_page.amap_msg = format!("规划失败：{e}");
                    self.status = self.run_page.amap_msg.clone();
                }
            }
        }

        if !self.run_page.amap_plan_requested {
            return;
        }
        self.run_page.amap_plan_requested = false;
        if self.run_page.amap_busy {
            return;
        }
        // 高德模式：路径点为空时用跑步检查点（实时点位）自动填充，失败则中止本次规划。
        if self.run_page.custom_points_bd.is_none() && !self.amap_fill_from_checkpoints() {
            return;
        }
        let seq = match self.run_page.amap_sequence_gcj() {
            Ok(s) => s,
            Err(e) => {
                self.run_page.amap_msg = e;
                return;
            }
        };
        // 持久化凭据与走法，便于下次直接使用。
        self.config.amap_key = self.run_page.amap_key.clone();
        self.config.amap_security_js_code = self.run_page.amap_jscode.clone();
        self.config.custom_close = self.run_page.custom_close.as_str().into();
        let _ = crate::api::model::save_config(&self.config);

        let cfg = crate::api::amap::AmapConfig {
            key: self.run_page.amap_key.clone(),
            jscode: self.run_page.amap_jscode.clone(),
        };
        self.run_page.amap_busy = true;
        self.run_page.amap_msg = "高德步行规划中…".into();
        self.status = self.run_page.amap_msg.clone();
        let tx = self.amap_tx.clone();
        std::thread::spawn(move || {
            let mut logs: Vec<String> = Vec::new();
            let res = {
                let mut lg = |s: &str| logs.push(s.to_string());
                crate::api::amap::plan_walking(&cfg, &seq, &mut lg)
            };
            let payload = match res {
                Ok(r) => Ok((r.points_bd, r.length_m, logs)),
                Err(e) => Err((e, logs)),
            };
            let _ = tx.send(payload);
        });
    }

    /// 用跑步检查点（实时点位缓存）填充高德路径点。
    ///
    /// 返回 true 表示当前已有可用路径点；false 表示无法继续（已在 `amap_msg` 写明原因）。
    pub(crate) fn amap_fill_from_checkpoints(&mut self) -> bool {
        let cached = self
            .identity
            .anchor_coordinate()
            .ok()
            .and_then(crate::api::model::load_points_cache_for);
        match cached {
            Some((_ts, pts)) => {
                let pts_bd = crate::api::points::points_bd(&pts);
                if pts_bd.len() < 2 {
                    self.run_page.amap_msg = "检查点不足 2 个，请稍后重试或手动填路径点".into();
                    return false;
                }
                // 按质心角排序使环序自然；坐标已是 BD-09，基准同步为 BD-09。
                let ordered = crate::track::generate_road::radial_order(&pts_bd);
                let text = ordered
                    .iter()
                    .map(|(la, lo)| format!("{la},{lo}"))
                    .collect::<Vec<_>>()
                    .join("\n");
                self.run_page.custom_datum = crate::track::custom::Datum::Bd09;
                self.run_page.custom_points_bd = Some(ordered);
                self.run_page.custom_text = text;
                self.run_page.custom_text_seen = self.run_page.custom_text.clone();
                self.run_page.custom_dirty = false;
                self.run_page.custom_msg =
                    format!("√ 已自动获取检查点 {} 个用于高德规划", pts_bd.len());
                self.status = self.run_page.custom_msg.clone();
                true
            }
            None => {
                // 无检查点缓存：能拉取则触发后台拉取，否则提示先登录/配锚点。
                if self.session.is_some() && !self.identity.has_unconfigured_default_location() {
                    self.refresh_points();
                    self.run_page.amap_msg =
                        "正在获取检查点…请稍候再点「获取检查点并规划」".into();
                } else {
                    self.run_page.amap_msg = "请先登录并配置定位锚点，或手动填写路径点".into();
                }
                false
            }
        }
    }

    fn draw_run_map(&mut self, ui: &mut egui::Ui) {
        if self.run_page.route_mode == RouteMode::Legacy {
            return;
        }
        if self.run_page.route_mode == RouteMode::Road && self.network.is_none() {
            ui.colored_label(
                theme::warn(),
                "未加载 OSM 路网，无法预览（请先到「路网」页导入）",
            );
            return;
        }
        let Some(plan) = self.run_page.plan.clone() else {
            return;
        };

        if self.run_page.preview_stale {
            self.run_page.preview = None;
            self.run_page.map_fitted = false;
            self.run_page.preview_stale = false;
            if self.run_page.route_mode.is_polyline_based() {
                match self.run_page.active_points_bd() {
                    Some(pts) if pts.len() >= 2 => {
                        let bld_path = if self.run_page.custom_use_buildings {
                            self.config.osm_path.clone()
                        } else {
                            String::new()
                        };
                        let buildings = crate::track::generate_road::load_buildings_bd(&bld_path);
                        match crate::track::generate_road::plan_custom_view(
                            &pts,
                            self.run_page.custom_close,
                            &buildings,
                        ) {
                            Ok(p) => self.run_page.preview = Some(p),
                            Err(e) => self.status = format!("路径预览失败：{e}"),
                        }
                    }
                    _ => {
                        self.status = if self.run_page.route_mode == RouteMode::Amap {
                            "高德路径尚未规划，请先点「规划道路」".into()
                        } else {
                            "自定义路径未解析出有效点".into()
                        }
                    }
                }
            } else if let Some(net) = self.network.clone() {
                // 点位缓存按锚点隔离（main 的串城市防护）；锚点无效时视为无缓存
                let cached = self
                    .identity
                    .anchor_coordinate()
                    .ok()
                    .and_then(crate::api::model::load_points_cache_for);
                if let Some((_ts, pts)) = cached {
                    let pts_bd = crate::api::points::points_bd(&pts);
                    if !pts_bd.is_empty() {
                        let fences = crate::api::model::load_fence_cache().unwrap_or_default();
                        match crate::track::generate_road::plan_road_view(
                            &net,
                            &pts_bd,
                            plan.dist * 1000.0,
                            plan.seed,
                            &fences,
                        ) {
                            Ok(p) => self.run_page.preview = Some(p),
                            Err(e) => self.status = format!("路线预览失败：{e}"),
                        }
                    } else {
                        self.status = "无打卡点缓存，提交后可回显轨迹".into();
                    }
                } else {
                    self.status = "无打卡点缓存，提交后可回显轨迹".into();
                }
            }
        }

        if self.run_page.preview.is_some() {
            if self.run_page.route_mode.is_polyline_based() {
                let mut items: Vec<(&str, egui::Color32)> = vec![
                    ("路线", egui::Color32::from_rgb(30, 111, 216)),
                    ("途经点", egui::Color32::from_rgb(240, 180, 0)),
                    ("起点", egui::Color32::from_rgb(22, 160, 90)),
                    ("终点", egui::Color32::from_rgb(220, 38, 38)),
                ];
                if self.run_page.custom_use_buildings && self.network.is_some() {
                    items.insert(1, ("建筑", egui::Color32::from_rgb(224, 194, 170)));
                }
                super::map::legend(ui, &items);
            } else {
                super::map::legend(
                    ui,
                    &[
                        ("道路", egui::Color32::from_rgb(200, 208, 204)),
                        ("建筑", egui::Color32::from_rgb(224, 194, 170)),
                        ("路线", egui::Color32::from_rgb(30, 111, 216)),
                        ("打卡点", egui::Color32::from_rgb(240, 180, 0)),
                        ("起点", egui::Color32::from_rgb(22, 160, 90)),
                        ("终点", egui::Color32::from_rgb(220, 38, 38)),
                    ],
                );
            }
        }

        ui.add_space(4.0);
        let rect = ui.available_rect_before_wrap();
        let (response, painter) = ui.allocate_painter(
            egui::Vec2::new(rect.width().max(200.0), 240.0),
            egui::Sense::drag(),
        );
        let canvas = response.rect;
        painter.rect_filled(
            canvas,
            egui::Rounding::ZERO,
            egui::Color32::from_rgb(250, 252, 251),
        );

        // 首次预览后适配视野：优先以电子围栏为中点/范围，无围栏时退化为路线+打卡点+道路
        if !self.run_page.map_fitted {
            let mut bounds: Vec<(f64, f64)> = Vec::new();
            if let Some(p) = &self.run_page.preview {
                for f in &p.fences {
                    bounds.extend(f.iter().copied());
                }
                if bounds.is_empty() {
                    bounds = super::map::collect_bounds(&p.edges);
                    bounds.extend(p.route.iter().copied());
                    bounds.extend(p.checkpoints.iter().copied());
                }
            }
            if !bounds.is_empty() {
                self.run_page.map.fit(canvas, &bounds);
                self.run_page.map_fitted = true;
            }
        }

        if let Some(p) = &self.run_page.preview {
            let road = egui::Color32::from_rgb(200, 208, 204);
            for e in &p.edges {
                self.run_page
                    .map
                    .draw_polyline(&painter, canvas, e, road, 1.0);
            }
            let bld = egui::Color32::from_rgb(224, 194, 170);
            for b in &p.buildings {
                self.run_page
                    .map
                    .draw_polygon(&painter, canvas, b, bld, 1.0);
            }
            let fence_c = egui::Color32::from_rgb(180, 118, 0);
            for f in &p.fences {
                self.run_page
                    .map
                    .draw_polygon(&painter, canvas, f, fence_c, 2.0);
            }
            let route_c = egui::Color32::from_rgb(30, 111, 216);
            self.run_page
                .map
                .draw_polyline(&painter, canvas, &p.route, route_c, 2.5);
            let cp = egui::Color32::from_rgb(240, 180, 0);
            for &(la, lo) in &p.checkpoints {
                self.run_page
                    .map
                    .draw_point(&painter, canvas, la, lo, cp, 3.5);
            }
            if let Some(first) = p.route.first() {
                self.run_page.map.draw_point(
                    &painter,
                    canvas,
                    first.0,
                    first.1,
                    egui::Color32::from_rgb(22, 160, 90),
                    4.5,
                );
            }
            if let Some(last) = p.route.last() {
                self.run_page.map.draw_point(
                    &painter,
                    canvas,
                    last.0,
                    last.1,
                    egui::Color32::from_rgb(220, 38, 38),
                    4.5,
                );
            }
            let custom = self.run_page.route_mode.is_polyline_based();
            let pt_name = if custom {
                self.run_page.custom_close.label()
            } else {
                "打卡点"
            };
            let label = if p.length_m > 0.0 {
                let loops = plan.dist * 1000.0 / p.length_m;
                if loops > 1.15 {
                    format!(
                        "本次方案 {:.2} km · 单圈 {:.0} m × {:.1} 圈 · {} {pt_name}",
                        plan.dist,
                        p.length_m,
                        loops,
                        p.checkpoints.len()
                    )
                } else {
                    format!(
                        "本次方案 {:.2} km · 路线 {:.0} m · {} {pt_name}",
                        plan.dist,
                        p.length_m,
                        p.checkpoints.len()
                    )
                }
            } else {
                format!(
                    "本次方案 {:.2} km · {} {pt_name}",
                    plan.dist,
                    p.checkpoints.len()
                )
            };
            painter.text(
                egui::Pos2::new(canvas.left() + 8.0, canvas.top() + 8.0),
                egui::Align2::LEFT_TOP,
                label,
                egui::FontId::proportional(12.0),
                egui::Color32::from_rgb(88, 104, 99),
            );
        } else {
            let hint = if self.run_page.route_mode == RouteMode::Amap {
                "（高德路径尚未规划，请先点「规划道路」）"
            } else if self.run_page.route_mode == RouteMode::Custom {
                "（自定义路径未解析出有效点）"
            } else {
                "（无预览）"
            };
            painter.text(
                canvas.center(),
                egui::Align2::CENTER_CENTER,
                hint,
                egui::FontId::proportional(14.0),
                egui::Color32::from_rgb(150, 160, 156),
            );
        }

        self.run_page.map.interact(ui, canvas);
    }

    fn start_run(&mut self) {
        // 参数变更时补齐方案；提交直接使用预计算值
        self.run_page.ensure_plan();
        self.run_page.ensure_custom_parsed();
        let page = &mut self.run_page;
        let plan = match page.plan.clone() {
            Some(p) => p,
            None => return,
        };
        // 折线路径（自定义 / 高德）：不可用时阻断提交，避免静默回退到经典算法。
        if page.route_mode.is_polyline_based() && !page.route_ready() {
            self.status = format!("路径不可用：{}", page.route_block_msg());
            return;
        }
        let altitude_spec = if !page.manual_altitude_on {
            Ok(None)
        } else {
            let (lo, hi) = (
                page.manual_altitude_min.min(page.manual_altitude_max),
                page.manual_altitude_min.max(page.manual_altitude_max),
            );
            // 两框相同按固定海拔；否则按区间（parse_fields 统一做非负/上界校验）
            if (hi - lo).abs() < 0.05 {
                crate::track::altitude::parse_fields(&format!("{lo}"), "")
            } else {
                crate::track::altitude::parse_fields(&format!("{lo}"), &format!("{hi}"))
            }
        };
        let altitude_spec = match altitude_spec {
            Ok(spec) => spec,
            Err(e) => {
                self.status = e;
                return;
            }
        };
        let (manual_altitude, manual_altitude_range) = match altitude_spec {
            None => (None, None),
            Some(crate::track::altitude::AltitudeSpec::Single(value)) => (Some(value), None),
            Some(crate::track::altitude::AltitudeSpec::Range(range)) => (None, Some(range)),
        };
        let (dist, dur) = (plan.dist * 1000.0, plan.dur); // 米
        let start_ms = plan.start_ms;
        let face_check = if page.face_check { 1 } else { 0 };
        let route_mode = page.route_mode;
        let gps_drift_m = page.gps_drift_m as f64;
        let custom_route = if route_mode.is_polyline_based() {
            Some(page.custom_route_payload(&self.config.osm_path))
        } else {
            None
        };
        self.config.dist_min = page.dist_min;
        self.config.dist_max = page.dist_max;
        self.config.pace_min = page.pace_min;
        self.config.pace_max = page.pace_max;
        self.config.face_check = page.face_check;
        self.config.manual_altitude = manual_altitude;
        self.config.manual_altitude_range = manual_altitude_range;
        self.config.route_mode = route_mode.as_str().into();
        self.config.custom_route_path = page.custom_path.clone();
        self.config.custom_datum = page.custom_datum.as_str().into();
        self.config.custom_use_buildings = page.custom_use_buildings;
        self.config.gps_drift_m = page.gps_drift_m;
        let _ = crate::api::model::save_config(&self.config);

        let identity = self.identity.clone();
        let session = match self.session.clone() {
            Some(s) => s,
            None => {
                self.status = "请先登录".into();
                return;
            }
        };
        self.run_busy = true;
        self.status = "跑步提交中…".into();
        let seed = plan.seed;
        self.spawn_job(move |tx| {
            let mut log = App::logger(tx.clone());
            let mut client = crate::api::client::ApiClient::new(identity, Some(session));
            let params = crate::api::flow::RunParams {
                dist,
                dur,
                start_ms,
                face_check,
                manual_altitude,
                manual_altitude_range,
                seed,
                route_mode,
                custom_route,
                gps_drift_m,
            };
            let payload = match crate::api::flow::run_full_flow(&mut client, &params, &mut log) {
                Ok(out) => {
                    log(&format!(
                        "全链完成 rrid={} obs={}/2 verify={} uuid={}",
                        out.result.rrid, out.obs_ok, out.detail_ok, out.result.uuid
                    ));
                    serde_json::json!({
                        "ok": true, "rrid": out.result.rrid,
                        "obs_ok": out.obs_ok, "verify": out.detail_ok,
                        "uuid": out.result.uuid,
                        "dist": out.result.total_dis, "dur": out.result.total_time,
                        "steps": out.result.total_steps, "avg_step_freq": out.result.avg_step_freq,
                        "calorie": out.result.calorie, "avg_power": out.result.avg_power,
                        "sel_distance": out.result.sel_distance, "start": out.result.start_ms,
                    })
                }
                Err(e) => {
                    log(&format!("跑步提交失败: {e}"));
                    serde_json::json!({ "ok": false, "message": e })
                }
            };
            tx.send(format!("__RUN_DONE__{payload}")).ok();
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 一份参数合理的页面状态（配速 6:00/km，距离 2 km 附近）。
    fn page(start_mode: usize, days_ago: i64) -> RunPage {
        RunPage {
            dist_min: 2.0,
            dist_max: 2.2,
            pace_min: 350.0,
            pace_max: 370.0,
            start_mode,
            days_ago,
            hour: 12,
            minute: 0,
            face_check: false,
            plan: None,
            ..Default::default()
        }
    }

    /// 开跑时刻本身不得落在未来（两种模式都成立）。
    fn assert_start_not_future(p: &RunPlan) {
        let now = crate::crypto::envelope::now_ms();
        assert!(p.start_ms <= now, "开始时刻 {} 落在未来", p.start_ms);
    }

    /// 随机模式：整段跑步（含 flow.rs 的 0-4s 抖动）都不得越过当前时刻。
    ///
    /// 指定模式不适用：用户指定今天 12:00 而此刻已 12:05 时，原行为就是照常提交。
    fn assert_random_not_future(p: &RunPlan) {
        let now = crate::crypto::envelope::now_ms();
        assert!(
            p.start_ms + p.dur * 1000 + JITTER_MARGIN_MS <= now,
            "随机时刻 {} + 用时 {}s 越过当前时刻",
            p.start_ms,
            p.dur
        );
    }

    /// 指定模式下「换一版」只换运动量，用户填的时刻必须原样保留。
    #[test]
    fn shuffle_keeps_user_time_in_specified_mode() {
        let mut p = page(1, 1);
        p.hour = 9;
        p.minute = 15;
        p.ensure_plan();
        for _ in 0..40 {
            p.regen_plan();
            let plan = p.plan.clone().unwrap();
            assert_eq!((p.hour, p.minute), (9, 15), "「换一版」改动了用户填的时刻");
            assert_eq!((plan.hour, plan.minute), (9, 15), "方案时刻与输入框不一致");
        }
    }

    /// 指定模式下「换一版」仍应换掉距离/配速（否则按钮看起来没反应）。
    #[test]
    fn shuffle_changes_shape_in_specified_mode() {
        let mut p = page(1, 1);
        p.hour = 9;
        p.minute = 15;
        p.ensure_plan();
        let mut seen = std::collections::HashSet::new();
        for _ in 0..40 {
            p.regen_plan();
            seen.insert(p.plan.as_ref().unwrap().dist.to_bits());
        }
        assert!(seen.len() > 1, "「换一版」40 次仍未改变距离");
    }

    /// 日期由用户决定，「换一版」不应改动它。
    #[test]
    fn shuffle_keeps_days_ago_in_specified_mode() {
        let mut p = page(1, 2);
        for _ in 0..20 {
            p.regen_plan();
            assert_eq!(p.days_ago, 2);
            assert_eq!(p.plan.as_ref().unwrap().days_ago, 2);
        }
    }

    /// 随机模式：时刻应落在该日 7:00-20:00 内，且「换一版」每次都换新的。
    #[test]
    fn random_mode_within_window_and_varies() {
        let mut p = page(0, 1);
        let mut seen = std::collections::HashSet::new();
        for _ in 0..40 {
            p.regen_plan();
            let plan = p.plan.clone().unwrap();
            let t = Local.timestamp_millis_opt(plan.start_ms).single().unwrap();
            assert!(
                (RAND_HOUR_LO..=RAND_HOUR_HI).contains(&t.hour()),
                "随机时刻 {}:{} 越出 {RAND_HOUR_LO}:00-{RAND_HOUR_HI}:00",
                t.hour(),
                t.minute()
            );
            seen.insert((t.hour(), t.minute()));
        }
        assert!(seen.len() > 1, "随机模式 40 次仍未改变时刻");
    }

    /// 任何模式下开始时刻都不得落在未来；随机模式还要求整段跑步不越过当前时刻。
    #[test]
    fn never_schedules_future_start() {
        for mode in [0, 1] {
            for days_ago in 0..=3 {
                let mut p = page(mode, days_ago);
                for _ in 0..25 {
                    p.regen_plan();
                    assert_start_not_future(p.plan.as_ref().unwrap());
                    if mode == 0 {
                        assert_random_not_future(p.plan.as_ref().unwrap());
                    }
                }
                // 改参数走 ensure_plan 的路径同样成立
                p.dist_max = 5.0;
                p.pace_max = 500.0;
                p.ensure_plan();
                let plan = p.plan.as_ref().unwrap();
                assert_start_not_future(plan);
                if mode == 0 {
                    assert_random_not_future(plan);
                }
            }
        }
    }

    /// 今天 + 已过去的指定时刻：按原时刻提交，不动它。
    #[test]
    fn specified_today_past_hour_is_kept() {
        let mut p = page(1, 0);
        p.regen_plan();
        // 用户在界面上填时刻（regen_plan 之后填，才是用户输入而非抽样结果）
        p.hour = 0;
        p.minute = 30;
        p.ensure_plan();
        let plan = p.plan.clone().unwrap();
        let today = Local::now().date_naive();
        let want = Local
            .with_ymd_and_hms(today.year(), today.month(), today.day(), 0, 30, 0)
            .single()
            .unwrap()
            .timestamp_millis();
        // 凌晨 0:30 尚未到达时会被钳制，跳过断言
        if want <= crate::crypto::envelope::now_ms() {
            assert_eq!(plan.start_ms, want, "已过去的指定时刻不应被改动");
        }
        assert_start_not_future(&plan);
    }

    /// 未来的指定时刻被钳制到当前时刻；已过去的时刻原样保留。
    ///
    /// 断言直接对齐契约 `min(填入时刻, 现在)`，与运行时刻无关（23:59 跑也不会误报）。
    #[test]
    fn specified_time_is_clamped_to_now() {
        let now = Local::now();
        let mut p = page(1, 0);
        p.hour = 23;
        p.minute = 59;
        p.regen_plan();
        let plan = p.plan.clone().unwrap();

        let today = now.date_naive();
        let want = Local
            .with_ymd_and_hms(today.year(), today.month(), today.day(), 23, 59, 0)
            .single()
            .unwrap()
            .timestamp_millis();
        let now_ms = crate::crypto::envelope::now_ms();
        if want <= now_ms {
            assert_eq!(plan.start_ms, want, "已过去的指定时刻应原样保留");
        } else {
            // 指定时刻仍在未来：start_ms 取「生成计划那一刻」的 now，与断言时刻存在毫秒级误差。
            assert!(
                plan.start_ms <= now_ms && now_ms - plan.start_ms < 1000,
                "未来的指定时刻应钳制到生成时的当前时刻：start_ms={} now={}",
                plan.start_ms,
                now_ms
            );
        }
        assert_start_not_future(&plan);
    }

    /// 改距离/配速只重抽运动量，不应打乱用户指定的时刻。
    #[test]
    fn shape_change_keeps_specified_time() {
        let mut p = page(1, 1);
        p.regen_plan();
        // 用户在界面上填时刻
        p.hour = 9;
        p.minute = 15;
        p.ensure_plan();
        let before = p.plan.clone().unwrap();
        assert_eq!((before.hour, before.minute), (9, 15));

        p.dist_min = 3.0;
        p.dist_max = 3.5;
        p.ensure_plan();
        let after = p.plan.clone().unwrap();
        assert_ne!(
            (after.dist, after.pace),
            (before.dist, before.pace),
            "运动量未重抽"
        );
        assert_eq!(after.start_ms, before.start_ms, "改距离不应改动指定时刻");
        assert_eq!((p.hour, p.minute), (9, 15));
    }

    /// 改小时/分钟应立刻反映到方案，且不重抽运动量。
    #[test]
    fn editing_hhmm_resyncs_plan() {
        let mut p = page(1, 1);
        p.regen_plan();
        let before = p.plan.clone().unwrap();

        p.hour = 8;
        p.minute = 45;
        p.ensure_plan();
        let after = p.plan.clone().unwrap();
        assert_eq!((after.hour, after.minute), (8, 45));
        assert_eq!(
            (after.dist, after.pace, after.dur),
            (before.dist, before.pace, before.dur)
        );
        assert_start_not_future(&after);
    }

    /// 切换模式应立即换掉时刻，且不重抽运动量。
    #[test]
    fn mode_switch_resyncs_time_only() {
        let mut p = page(1, 1);
        p.regen_plan();
        let before = p.plan.clone().unwrap();

        p.start_mode = 0;
        p.ensure_plan();
        let after = p.plan.clone().unwrap();
        assert_eq!(after.start_mode, 0);
        assert_eq!(
            (after.dist, after.pace, after.dur),
            (before.dist, before.pace, before.dur)
        );
        assert_random_not_future(&after);
    }

    /// 日期下拉作用于两种模式。
    #[test]
    fn date_change_applies_to_both_modes() {
        for mode in [0, 1] {
            let mut p = page(mode, 0);
            p.regen_plan();
            p.days_ago = 3;
            p.ensure_plan();
            let plan = p.plan.clone().unwrap();
            assert_eq!(plan.days_ago, 3);
            let want = (Local::now() - Duration::days(3)).date_naive();
            let t = Local.timestamp_millis_opt(plan.start_ms).single().unwrap();
            assert_eq!(t.date_naive(), want, "mode={mode} 未落到 3 天前");
        }
    }

    /// 日期下拉文本。
    #[test]
    fn days_ago_labels() {
        assert_eq!(days_ago_label(0), "今天");
        assert_eq!(days_ago_label(1), "昨天");
        assert_eq!(days_ago_label(3), "3 天前");
    }

    /// 同一帧内「距离 + 日期」一起改：两处变化都不能漏。
    #[test]
    fn simultaneous_shape_and_date_change() {
        let mut p = page(0, 0);
        p.regen_plan();
        p.dist_min = 4.0;
        p.dist_max = 4.5;
        p.days_ago = 2;
        p.ensure_plan();
        let plan = p.plan.clone().unwrap();
        assert_eq!(plan.days_ago, 2, "日期变化被漏掉");
        assert!(plan.dist >= 4.0, "距离变化被漏掉");
        let want = (Local::now() - Duration::days(2)).date_naive();
        let t = Local.timestamp_millis_opt(plan.start_ms).single().unwrap();
        assert_eq!(t.date_naive(), want);
        assert_random_not_future(&plan);
    }

    /// 同一帧内「时刻 + 距离」一起改。
    #[test]
    fn simultaneous_shape_and_hhmm_change() {
        let mut p = page(1, 1);
        p.regen_plan();
        p.dist_min = 4.0;
        p.dist_max = 4.5;
        p.hour = 8;
        p.minute = 20;
        p.ensure_plan();
        let plan = p.plan.clone().unwrap();
        assert_eq!((plan.hour, plan.minute), (8, 20), "时刻变化被漏掉");
        assert!(plan.dist >= 4.0, "距离变化被漏掉");
        assert_eq!(plan.start_ms, specified_time(1, 8, 20));
        assert_start_not_future(&plan);
    }

    /// 今天 + 指定时刻且该时刻已过：照常提交，不再回退（用户指定的时刻说了算）。
    #[test]
    fn specified_past_time_is_committed_as_is() {
        let now = Local::now();
        // 取一个今天已过去足够久的整点，保证整段用时都已结束
        let h = now.hour().saturating_sub(3) as i64;
        let mut p = page(1, 0);
        p.hour = h;
        p.minute = 0;
        p.regen_plan();
        let plan = p.plan.clone().unwrap();
        let today = now.date_naive();
        let want = Local
            .with_ymd_and_hms(today.year(), today.month(), today.day(), h as u32, 0, 0)
            .single()
            .unwrap()
            .timestamp_millis();
        assert_eq!(plan.start_ms, want, "用户填的过去时刻被改动了");
        assert_start_not_future(&plan);
    }

    /// 明天（未来）的指定时刻必然被钳制，且界面可据此提示。
    ///
    /// 用「未来某天」构造，避免依赖当前钟点（23:xx 跑也不会失效）。
    #[test]
    fn future_specified_time_is_clamped_and_detectable() {
        // days_ago 取负即未来，但会被 clamp 到 0；这里直接构造今天最后一刻
        let mut p = page(1, 0);
        p.hour = 23;
        p.minute = 59;
        p.ensure_plan();
        let plan = p.plan.clone().unwrap();

        let now = Local::now();
        let today = now.date_naive();
        let typed = Local
            .with_ymd_and_hms(today.year(), today.month(), today.day(), 23, 59, 0)
            .single()
            .unwrap()
            .timestamp_millis();
        let now_ms = crate::crypto::envelope::now_ms();
        assert_eq!(plan.start_ms, typed.min(now_ms));
        assert_start_not_future(&plan);
        // 界面提示条件：填入时刻晚于现在（23:59 总成立，除非正好在那一分钟）
        if typed > now_ms {
            assert!(plan.start_ms < typed, "未来时刻未被钳制");
        }
    }

    /// 反复调用 ensure_plan 应当是幂等的（不改动任何已定值）。
    #[test]
    fn ensure_plan_is_idempotent() {
        for mode in [0, 1] {
            let mut p = page(mode, 1);
            p.regen_plan();
            let first = p.plan.clone().unwrap();
            for _ in 0..5 {
                p.ensure_plan();
            }
            let again = p.plan.clone().unwrap();
            assert_eq!(
                (again.dist, again.pace, again.dur, again.start_ms),
                (first.dist, first.pace, first.dur, first.start_ms),
                "mode={mode} ensure_plan 不稳定"
            );
        }
    }

    /// 随机模式抽样结果应落在所选日期的窗口内；凌晨选「今天」时窗口未到，
    /// 此时允许退化，但必须能被 random_window_ok 识别出来（界面据此提示）。
    #[test]
    fn random_window_detection() {
        let mut p = page(0, 1);
        p.regen_plan();
        assert!(p.random_window_ok(), "昨天的随机时刻应当落在窗口内");

        // 构造一个「今天但早于 7:00」的方案，窗口判定应为 false
        let now = Local::now();
        let early = Local
            .with_ymd_and_hms(now.year(), now.month(), now.day(), 3, 0, 0)
            .single()
            .unwrap()
            .timestamp_millis();
        p.days_ago = 0;
        if let Some(plan) = p.plan.as_mut() {
            plan.days_ago = 0;
            plan.start_ms = early;
        }
        assert!(
            !p.random_window_ok(),
            "今天 03:00 不应被判为在 7:00-20:00 窗口内"
        );
    }
}
