use std::{
    collections::VecDeque,
    sync::{Arc, RwLock},
};

use cursive::{
    Printer, Rect, Vec2, View,
    event::{Callback, Event, EventResult, MouseButton, MouseEvent},
    theme::{Color, ColorStyle, Style},
    utils::markup::StyledString,
    view::scroll,
    views::{NamedView, ScrollView},
};
use log::info;
use pad::PadStr as _;

use crate::monitor::{
    config::MonitorConfig,
    log::log_level_foreground_color,
    target_log::{DefmtLogInfo, TargetLog},
};

struct LogRow {
    log: TargetLog,
    log_content_offset: usize,
    show_line_number: bool,
    matches_filter: bool,
}

impl LogRow {
    fn new(log: TargetLog, config: &MonitorConfig) -> Self {
        Self {
            log_content_offset: if log.defmt.is_some() { 23 } else { 8 },
            matches_filter: config.matches(&log),
            log,
            show_line_number: false,
        }
    }

    fn calculate_height(&self, width: usize) -> usize {
        if width <= self.log_content_offset || self.log.log_content.len() == 0 {
            return 1;
        }

        let log_content_width = width - self.log_content_offset;
        let mut height = (self.log.log_content.len() - 1) / log_content_width + 1;

        if self.show_line_number {
            height += 1;
        }

        return height;
    }

    fn draw(&self, printer: &Printer) {
        let bg = self.log.node_type.background_color();
        printer.with_color(ColorStyle::new(Color::Rgb(0, 0, 0), bg), |printer| {
            for y in 0..printer.size.y {
                printer.print_hline((0, y), printer.size.x, " ");
            }

            printer.print((0, 0), &self.log.node_type.short_name().pad_to_width(4));
            printer.print(
                (4, 0),
                &self
                    .log
                    .node_id
                    .map_or(String::from("xxx"), |id| format!("{:0>3X}", id)),
            );

            if let Some(defmt_info) = &self.log.defmt {
                printer.print_styled(
                    (8, 0),
                    &StyledString::single_span(
                        defmt_info.log_level.to_string().pad_to_width(6),
                        Style::from_color_style(ColorStyle::front(log_level_foreground_color(
                            defmt_info.log_level,
                        ))),
                    ),
                );
                let timestamp = defmt_info
                    .timestamp
                    .map_or(String::new(), |t| format!("{:>8.3}", t));
                printer.print_styled(
                    (14, 0),
                    &StyledString::single_span(
                        timestamp,
                        Style::from_color_style(ColorStyle::front(Color::Rgb(100, 100, 100))),
                    ),
                );
            }

            if printer.size.x > self.log_content_offset {
                let log_content_width = printer.size.x - self.log_content_offset;
                let mut i = 0;
                for y in 0..(printer.size.y - if self.show_line_number { 1 } else { 0 }) {
                    printer.print(
                        (self.log_content_offset, y),
                        &self.log.log_content
                            [i..(i + log_content_width).min(self.log.log_content.len())],
                    );
                    i += log_content_width;
                }
            }

            if self.show_line_number {
                if let Some(DefmtLogInfo {
                    location: Some(location),
                    ..
                }) = &self.log.defmt
                {
                    printer.print(
                        (0, printer.size.y - 1),
                        &format!(
                            "└─ {} @ {}:{}",
                            location.module_path, location.file_path, location.line_number
                        ),
                    );
                } else {
                    printer.print((0, printer.size.y - 1), "└─ Line number info not avaliable");
                }
            }
        });
    }
}

struct HeightsCache {
    width: usize,
    heights: VecDeque<usize>,
}

impl HeightsCache {
    fn push_row_height(&mut self, row: &LogRow) {
        self.heights.push_back(row.calculate_height(self.width));
        if self.heights.len() > 500 {
            self.heights.pop_front();
        }
    }
}

pub struct LogsView {
    children: VecDeque<LogRow>,
    // more recent
    heights_cache_1: Box<HeightsCache>,
    heights_cache_2: Box<HeightsCache>,
    last_config: MonitorConfig,
    config: Arc<RwLock<MonitorConfig>>,
}

impl LogsView {
    pub fn new(config: Arc<RwLock<MonitorConfig>>) -> Self {
        Self {
            children: VecDeque::new(),
            heights_cache_1: Box::new(HeightsCache {
                width: 100,
                heights: VecDeque::new(),
            }),
            heights_cache_2: Box::new(HeightsCache {
                width: 102,
                heights: VecDeque::new(),
            }),
            last_config: { config.read().unwrap().clone() },
            config,
        }
    }

    pub fn push_log(&mut self, log: TargetLog) {
        let log_row = LogRow::new(log, &self.last_config);
        self.heights_cache_1.push_row_height(&log_row);
        self.heights_cache_2.push_row_height(&log_row);

        self.children.push_back(log_row);
        if self.children.len() > 500 {
            self.children.pop_front();
        }
    }
}

impl View for LogsView {
    fn draw(&self, printer: &Printer) {
        let mut y = 0usize;
        for (child, child_height) in self
            .children
            .iter()
            .zip(self.heights_cache_1.heights.iter())
        {
            if !child.matches_filter {
                continue;
            }

            if (y + child_height).saturating_sub(1) >= printer.content_offset.y
                && y <= printer.content_offset.y + printer.output_size.y
            {
                let sub_printer = printer.windowed(Rect::from_size(
                    Vec2::new(0, y),
                    Vec2::new(printer.output_size.x, *child_height),
                ));
                child.draw(&sub_printer);
            }

            y += *child_height;
        }
    }

    fn layout(&mut self, size: Vec2) {
        if size.x != self.heights_cache_1.width {
            std::mem::swap(&mut self.heights_cache_1, &mut self.heights_cache_2);
        }
    }

    fn required_size(&mut self, constraint: Vec2) -> Vec2 {
        if constraint.x == self.heights_cache_1.width {
            // do nothing
        } else if constraint.x == self.heights_cache_2.width {
            // swap cache 1 and cache 2
            std::mem::swap(&mut self.heights_cache_1, &mut self.heights_cache_2);
        } else {
            // move cache 1 to cache 2 and recalculate cache 1
            std::mem::swap(&mut self.heights_cache_1, &mut self.heights_cache_2);

            self.heights_cache_1.width = constraint.x;
            self.heights_cache_1.heights = self
                .children
                .iter()
                .map(|c| c.calculate_height(constraint.x))
                .collect();
        }

        let config = self.config.read().unwrap();
        if &*config != &self.last_config {
            info!("filter cache invalidated");
            for child in self.children.iter_mut() {
                child.matches_filter = config.matches(&child.log);
            }

            self.last_config = config.clone();
        }

        Vec2::new(
            constraint.x,
            self.children
                .iter()
                .zip(self.heights_cache_1.heights.iter())
                .filter(|(c, _)| c.matches_filter)
                .map(|(_, h)| *h)
                .sum(),
        )
    }

    fn on_event(&mut self, event: Event) -> EventResult {
        match event {
            Event::Mouse {
                offset,
                position,
                event: mouse_event,
            } => {
                if mouse_event == MouseEvent::WheelUp || mouse_event == MouseEvent::WheelDown {
                    EventResult::Consumed(Some(Callback::from_fn(move |s| {
                        let mut logs_scroll_view = s
                            .find_name::<ScrollView<NamedView<LogsView>>>("logs_scroll_view")
                            .unwrap();

                        scroll::on_event(
                            &mut *logs_scroll_view,
                            event.clone(),
                            |_, __| EventResult::Ignored,
                            |_, __| Rect::from_point(Vec2::zero()),
                        );
                    })))
                } else if mouse_event == MouseEvent::Release(MouseButton::Left) {
                    let clicked_y = (position - offset).y;
                    let mut y = 0usize;
                    for ((child, child_height), child_height_2) in self
                        .children
                        .iter_mut()
                        .zip(self.heights_cache_1.heights.iter_mut())
                        .zip(self.heights_cache_2.heights.iter_mut())
                    {
                        if !child.matches_filter {
                            continue;
                        }

                        y += *child_height;

                        if y > clicked_y {
                            if child.show_line_number {
                                child.show_line_number = false;
                                *child_height -= 1;
                                *child_height_2 -= 1;
                            } else {
                                child.show_line_number = true;
                                *child_height += 1;
                                *child_height_2 += 1;
                            }
                            break;
                        }
                    }

                    EventResult::Consumed(None)
                } else {
                    EventResult::Ignored
                }
            }
            _ => EventResult::Ignored,
        }
    }
}
