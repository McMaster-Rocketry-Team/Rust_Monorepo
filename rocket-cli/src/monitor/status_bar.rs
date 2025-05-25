use std::{
    ops::Range,
    sync::{Arc, RwLock},
};

use cursive::{
    Vec2, View,
    align::HAlign,
    direction::Direction,
    event::{Event, EventResult},
    theme::{BaseColor, ColorStyle, Effect, Effects, PaletteColor, Style},
    utils::{markup::StyledString, span::SpannedString},
    view::CannotFocus,
};
use pad::PadStr as _;
use tokio::sync::watch;

use super::MonitorStatus;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SelectedTab {
    LogViewer,
    CanMessageViewer,
    NodeStatus,
}

pub struct StatusBar {
    connection_method_name: String,
    status_rx: Arc<RwLock<watch::Receiver<MonitorStatus>>>,
    selected_tab: SelectedTab,
    click_ranges: Arc<RwLock<(Range<usize>, Range<usize>, Range<usize>)>>,
}

impl StatusBar {
    pub fn new(connection_method_name: String, status_rx: watch::Receiver<MonitorStatus>) -> Self {
        Self {
            connection_method_name,
            status_rx: Arc::new(RwLock::new(status_rx)),
            selected_tab: SelectedTab::LogViewer,
            click_ranges: Arc::new(RwLock::new((0..0, 0..0, 0..0))),
        }
    }

    pub fn selected_tab(&self) -> SelectedTab {
        self.selected_tab
    }
}

impl View for StatusBar {
    fn draw(&self, printer: &cursive::Printer) {
        let normal_style = Style {
            effects: Effects::only(Effect::Underline),
            color: ColorStyle::new(PaletteColor::Primary, PaletteColor::Background),
        };
        printer.print_styled(
            (0, 0),
            &StyledString::single_span("".pad_to_width(printer.size.x), normal_style),
        );

        printer.print_styled(
            (0, 0),
            &StyledString::single_span(&self.connection_method_name, normal_style),
        );

        let status_text = match self.status_rx.read().unwrap().borrow().clone() {
            MonitorStatus::Initialize => StyledString::single_span(
                "Initialize",
                Style {
                    effects: Effects::only(Effect::Underline),
                    color: ColorStyle::new(BaseColor::Blue.dark(), PaletteColor::Background),
                },
            ),
            MonitorStatus::Normal => StyledString::single_span(
                "Normal",
                Style {
                    effects: Effects::only(Effect::Underline),
                    color: ColorStyle::new(BaseColor::Green.dark(), PaletteColor::Background),
                },
            ),
            MonitorStatus::ChunkError => StyledString::single_span(
                "Malformed",
                Style {
                    effects: Effects::only(Effect::Underline),
                    color: ColorStyle::new(BaseColor::Red.dark(), PaletteColor::Background),
                },
            ),
            MonitorStatus::Overrun => StyledString::single_span(
                "Overrun",
                Style {
                    effects: Effects::only(Effect::Underline),
                    color: ColorStyle::new(BaseColor::Yellow.dark(), PaletteColor::Background),
                },
            ),
        };
        printer.print_styled((self.connection_method_name.len() + 1, 0), &status_text);

        printer.print_styled(
            (printer.size.x - 10, 0),
            &StyledString::single_span("rocket-cli", normal_style),
        );

        // Tab
        let selected_tab_style = Style {
            effects: Effects::default(),
            color: ColorStyle::new(BaseColor::White, BaseColor::Black),
        };

        let mut tab_string = SpannedString::new();
        tab_string.append_styled(
            "Logs",
            if self.selected_tab == SelectedTab::LogViewer {
                selected_tab_style
            } else {
                normal_style
            },
        );
        tab_string.append_styled(" ", normal_style);
        tab_string.append_styled(
            "Messages",
            if self.selected_tab == SelectedTab::CanMessageViewer {
                selected_tab_style
            } else {
                normal_style
            },
        );
        tab_string.append_styled(" ", normal_style);
        tab_string.append_styled(
            "Nodes",
            if self.selected_tab == SelectedTab::NodeStatus {
                selected_tab_style
            } else {
                normal_style
            },
        );

        let mut start_x = HAlign::Center.get_offset(tab_string.width(), printer.size.x);
        printer.print_styled((start_x, 0), &tab_string);
        let mut click_ranges = self.click_ranges.write().unwrap();
        click_ranges.0 = start_x..(start_x + 4);
        start_x += 4 + 1;
        click_ranges.1 = start_x..(start_x + 8);
        start_x += 8 + 1;
        click_ranges.2 = start_x..(start_x + 5);
    }

    fn on_event(&mut self, event: Event) -> EventResult {
        let click_ranges = self.click_ranges.write().unwrap();

        if let Event::Mouse {
            position, offset, ..
        } = event
        {
            if position.fits_in_rect(offset, Vec2::new(100000, 1)) {
                if click_ranges.0.contains(&position.x) {
                    self.selected_tab = SelectedTab::LogViewer;
                    return EventResult::consumed();
                } else if click_ranges.1.contains(&position.x) {
                    self.selected_tab = SelectedTab::CanMessageViewer;
                    return EventResult::consumed();
                } else if click_ranges.2.contains(&position.x) {
                    self.selected_tab = SelectedTab::NodeStatus;
                    return EventResult::consumed();
                }
            }
        }

        EventResult::Ignored
    }

    fn take_focus(&mut self, _: Direction) -> Result<EventResult, CannotFocus> {
        Ok(EventResult::Consumed(None))
    }
}
