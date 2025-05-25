use anyhow::Result;
use cursive::theme::Palette;

pub async fn can_message_viewer_tui()->Result<()> {
    let mut siv = cursive::default();
    let mut theme = siv.current_theme().clone();

    theme.palette = Palette::terminal_default();
    siv.set_theme(theme);

    Ok(())
}