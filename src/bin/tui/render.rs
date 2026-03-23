//! Ratatui layout and drawing.

use std::sync::{Arc, Mutex};

use ratatui::layout::{Constraint, Direction, Layout};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Borders, Clear, Paragraph, Wrap};
use ratatui::Frame;

use crate::format::{format_mode_short, format_uptime_ms, waypoint_line};
use crate::state::{OverrideState, TelemetryState};

/// Panel border/style colors for a distinct look per section.
fn vehicle_style() -> Style {
    Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)
}
fn attitude_style() -> Style {
    Style::default().fg(Color::Green).add_modifier(Modifier::BOLD)
}
fn gps_style() -> Style {
    Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD)
}
fn battery_style() -> Style {
    Style::default().fg(Color::Magenta).add_modifier(Modifier::BOLD)
}
fn hud_style() -> Style {
    Style::default().fg(Color::Blue).add_modifier(Modifier::BOLD)
}
fn mission_style() -> Style {
    Style::default().fg(Color::LightCyan).add_modifier(Modifier::BOLD)
}
fn messages_style() -> Style {
    Style::default().fg(Color::DarkGray).add_modifier(Modifier::BOLD)
}

pub(crate) fn draw_ui(
    f: &mut Frame,
    state: &TelemetryState,
    override_state: &Arc<Mutex<OverrideState>>,
    waypoint_input: Option<&str>,
) {
    // Side-by-side: left column (telemetry panels), right column (mission + messages)
    let main_chunks = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Percentage(38), Constraint::Percentage(62)])
        .split(f.area());

    let left = main_chunks[0];
    let right = main_chunks[1];

    // Left column: status bar + vertical stack of panels
    let left_chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(1),
            Constraint::Length(7),
            Constraint::Length(4),
            Constraint::Length(5),
            Constraint::Length(4),
            Constraint::Length(4),
        ])
        .split(left);

    let status_line = {
        let sys = state
            .vehicle_sysid
            .map(|u| u.to_string())
            .unwrap_or_else(|| "?".to_string());
        let mode = state
            .heartbeat_custom
            .map(format_mode_short)
            .unwrap_or("?");
        let armed_str = state
            .armed
            .map(|b| if b { "yes" } else { "no" })
            .unwrap_or("?");
        let override_str = if let Ok(os) = override_state.lock() {
            match &*os {
                OverrideState::MissionRunning => String::new(),
                OverrideState::Paused => "  PAUSED (c=resume)".to_string(),
                OverrideState::OverrideActive { waypoints, index, .. } => {
                    format!("  OVERRIDE {}/{}", index + 1, waypoints.len())
                }
                OverrideState::Resuming { .. } => "  RESUMING".to_string(),
            }
        } else {
            String::new()
        };
        let net_str = match state.net_online {
            Some(true) => format!(
                "  NET=UP ok:{}s chk:{}s",
                state.net_secs_since_last_ok.unwrap_or(0),
                state.net_secs_since_last_check.unwrap_or(0)
            ),
            Some(false) => format!(
                "  NET=DOWN {}s{} chk:{}s",
                state.net_offline_secs.unwrap_or(0),
                if state.net_rtl_sent_for_current_outage {
                    " RTL_SENT"
                } else {
                    ""
                },
                state.net_secs_since_last_check.unwrap_or(0)
            ),
            None => "  NET=CHECKING".to_string(),
        };
        format!(
            "SYS={}  MODE={}  ARMED={}{}{}",
            sys, mode, armed_str, net_str, override_str
        )
    };
    f.render_widget(
        Paragraph::new(Line::from(Span::raw(status_line))).wrap(Wrap { trim: true }),
        left_chunks[0],
    );

    let vehicle_lines: Vec<Line> = {
        let mut lines = Vec::new();
        lines.push(Line::from(Span::raw(format!(
            "SYS: {}  COMP: {}  TYPE: {}",
            state.vehicle_sysid.map(|u| u.to_string()).as_deref().unwrap_or("—"),
            state.vehicle_compid.map(|u| u.to_string()).as_deref().unwrap_or("—"),
            state.vehicle_type_name.as_deref().unwrap_or("—")
        ))));
        let armed_display = state
            .armed
            .map(|b| if b { "ARMED".to_string() } else { "false".to_string() })
            .unwrap_or_else(|| "—".to_string());
        let armed_style = if state.armed == Some(true) {
            Style::default().fg(Color::Red).add_modifier(Modifier::BOLD)
        } else {
            Style::default().fg(Color::DarkGray)
        };
        lines.push(Line::from(vec![
            Span::raw("MODE: "),
            Span::raw(state.vehicle_mode_name.as_deref().unwrap_or("—")),
            Span::raw("  ARMED: "),
            Span::styled(armed_display, armed_style),
        ]));
        lines.push(Line::from(Span::raw(format!(
            "Vbat: {:.2}V  Current: {}  Load: {}%  Uptime: {}",
            state.sys_voltage.unwrap_or(0.0),
            state.sys_current.map(|c| format!("{:.2}A", c)).as_deref().unwrap_or("—"),
            state.sys_load.map(|l| (l / 10).to_string()).as_deref().unwrap_or("—"),
            state.time_boot_ms.map(format_uptime_ms).as_deref().unwrap_or("—")
        ))));
        for s in &state.vehicle_info {
            lines.push(Line::from(Span::raw(s.as_str())));
        }
        lines
    };
    let vehicle_block = Block::default()
        .title(" Vehicle ")
        .borders(Borders::ALL)
        .border_style(vehicle_style());
    f.render_widget(
        Paragraph::new(vehicle_lines)
            .block(vehicle_block)
            .wrap(Wrap { trim: true }),
        left_chunks[1],
    );

    let att_line = format!(
        "Roll {:.1}°  Pitch {:.1}°  Yaw {:.1}°",
        state.roll.unwrap_or(0.0),
        state.pitch.unwrap_or(0.0),
        state.yaw.unwrap_or(0.0)
    );
    let att_block = Block::default()
        .title(" Attitude ")
        .borders(Borders::ALL)
        .border_style(attitude_style());
    f.render_widget(
        Paragraph::new(att_line).block(att_block).wrap(Wrap { trim: true }),
        left_chunks[2],
    );

    let home_str = match (state.home_lat, state.home_lon, state.home_alt) {
        (Some(lat), Some(lon), Some(alt)) => format!("Home: {:.6}, {:.6}, {:.1}m AMSL", lat, lon, alt),
        _ => "Home: —".to_string(),
    };
    let gps_pos_line = format!(
        "Fix {}  Sats {}  HDOP {}  |  Lat {:.6}  Lon {:.6}  Alt {:.1}m\n{}",
        state.gps_fix.as_deref().unwrap_or("—"),
        state.gps_sats.map(|u| u.to_string()).as_deref().unwrap_or("—"),
        state.gps_hdop.as_deref().unwrap_or("—"),
        state.lat.unwrap_or(0.0),
        state.lon.unwrap_or(0.0),
        state.alt.unwrap_or(0.0),
        home_str
    );
    let gps_block = Block::default()
        .title(" GPS / Position ")
        .borders(Borders::ALL)
        .border_style(gps_style());
    f.render_widget(
        Paragraph::new(gps_pos_line)
            .block(gps_block)
            .wrap(Wrap { trim: true }),
        left_chunks[3],
    );

    let bat_line = format!(
        "VBat {:.2}V  Batt {}  Cell1 {:.2}V",
        state.vbat.unwrap_or(0.0),
        state.batt_pct.as_deref().unwrap_or("—"),
        state.cell1_v.unwrap_or(0.0)
    );
    let bat_block = Block::default()
        .title(" Battery ")
        .borders(Borders::ALL)
        .border_style(battery_style());
    f.render_widget(
        Paragraph::new(bat_line).block(bat_block).wrap(Wrap { trim: true }),
        left_chunks[4],
    );

    let hud_line = format!(
        "Air {:.1}  Grd {:.1}  Hdg {}°  Thr {}  Climb {:.1}",
        state.airspeed.unwrap_or(0.0),
        state.groundspeed.unwrap_or(0.0),
        state.heading.unwrap_or(0),
        state.throttle.unwrap_or(0),
        state.climb.unwrap_or(0.0)
    );
    let hud_block = Block::default()
        .title(" HUD ")
        .borders(Borders::ALL)
        .border_style(hud_style());
    f.render_widget(
        Paragraph::new(hud_line).block(hud_block).wrap(Wrap { trim: true }),
        left_chunks[5],
    );

    // Right column: mission (fills space) + messages (fixed height)
    let right_chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Ratio(1, 3), Constraint::Ratio(2, 3)])
        .split(right);

    let mission_lines: Vec<Line> = if state.mission_waypoints.is_empty() {
        vec![Line::from(Span::raw("(no waypoints received)"))]
    } else {
        let header = Line::from(Span::styled(
            "(* = current WP)  alt: AMSL = above sea level, rel = relative to home",
            Style::default().fg(Color::DarkGray),
        ));
        let mut lines = vec![header];
        lines.extend(
            state
                .mission_waypoints
                .iter()
                .map(|w| {
                    let raw = waypoint_line(w, state.mission_current_seq);
                    let is_current = state.mission_current_seq == Some(w.seq);
                    Line::from(if is_current {
                        Span::styled(
                            raw,
                            Style::default()
                                .fg(Color::Yellow)
                                .add_modifier(Modifier::BOLD),
                        )
                    } else {
                        Span::raw(raw)
                    })
                }),
        );
        lines
    };
    let mission_block = Block::default()
        .title(" Mission (waypoints) ")
        .borders(Borders::ALL)
        .border_style(mission_style());
    let mission_area = right_chunks[0];
    let total_lines = mission_lines.len() as u16;
    let visible_lines = mission_area.height.saturating_sub(2); // inner height minus borders
    let scroll_offset = if total_lines <= visible_lines {
        0
    } else if let Some(seq) = state.mission_current_seq {
        let cur = 1 + seq as u16; // line index: 0 = header, 1 = wp0, ...
        let vis = visible_lines;
        let centered = cur.saturating_sub(vis / 2);
        centered.min(total_lines.saturating_sub(vis))
    } else {
        0
    };
    f.render_widget(
        Paragraph::new(mission_lines)
            .block(mission_block)
            .wrap(Wrap { trim: true })
            .scroll((scroll_offset, 0)),
        mission_area,
    );

    let msg_lines: Vec<Line> = if state.recent_messages.is_empty() {
        vec![Line::from(Span::raw("—"))]
    } else {
        state
            .recent_messages
            .iter()
            .map(|s| Line::from(Span::raw(s.as_str())))
            .collect()
    };
    let msg_block = Block::default()
        .title(" Messages [1]=TUI [2]=FC [3]=timeout | s=retry streams (h=help) ")
        .borders(Borders::ALL)
        .border_style(messages_style());
    f.render_widget(
        Paragraph::new(msg_lines)
            .block(msg_block)
            .wrap(Wrap { trim: true }),
        right_chunks[1],
    );

    if let Some(buf) = waypoint_input {
        let popup_w = 62_u16.min(f.area().width);
        let popup_h = 6_u16.min(f.area().height);
        let area = ratatui::layout::Rect {
            x: f.area().width.saturating_sub(popup_w) / 2,
            y: f.area().height.saturating_sub(popup_h) / 2,
            width: popup_w,
            height: popup_h,
        };
        f.render_widget(Clear, area);
        let block = Block::default()
            .title(" Override waypoint ")
            .borders(Borders::ALL)
            .border_style(Style::default().fg(Color::Yellow))
            .style(Style::default().bg(Color::Black));
        let inner = block.inner(area);
        f.render_widget(block, area);
        let popup_text = vec![
            Line::from(Span::styled(
                format!("  {}_", buf),
                Style::default().fg(Color::Yellow),
            )),
            Line::from(""),
            Line::from(Span::styled(
                "  lat lon alt (space-sep)  or  alt only.  Enter=go  Esc=cancel",
                Style::default().fg(Color::DarkGray),
            )),
        ];
        f.render_widget(
            Paragraph::new(popup_text).wrap(Wrap { trim: true }),
            inner,
        );
    }

    if state.show_help_popup {
        let help_bg = Color::White;
        let help_fg = Color::Black;
        let help_text = vec![
            Line::from(""),
            Line::from(Span::styled(
                " Keys (press h or Esc to close) ",
                Style::default()
                    .fg(Color::DarkGray)
                    .add_modifier(Modifier::BOLD),
            )),
            Line::from(""),
            Line::from(Span::styled("  q     Quit the TUI", Style::default().fg(help_fg))),
            Line::from(Span::styled(
                "  a     Arm motors (need GUIDED or armable mode)",
                Style::default().fg(help_fg),
            )),
            Line::from(Span::styled("  d     Disarm motors", Style::default().fg(help_fg))),
            Line::from(Span::styled(
                "  f     Force arm (bypasses some pre-arm checks)",
                Style::default().fg(help_fg),
            )),
            Line::from(Span::styled("  g     Set mode GUIDED", Style::default().fg(help_fg))),
            Line::from(Span::styled("  u     Set mode AUTO", Style::default().fg(help_fg))),
            Line::from(Span::styled(
                "  m     Set AUTO and start mission (follow",
                Style::default().fg(help_fg),
            )),
            Line::from(Span::styled("         waypoints)", Style::default().fg(help_fg))),
            Line::from(Span::styled(
                "  i     Interrupt: pause mission, hover here (c=resume)",
                Style::default().fg(help_fg),
            )),
            Line::from(Span::styled(
                "  w     Inject waypoint (during mission or when paused)",
                Style::default().fg(help_fg),
            )),
            Line::from(Span::styled(
                "         lat lon alt, or just alt. Then resume or stay paused",
                Style::default().fg(help_fg),
            )),
            Line::from(Span::styled(
                "  c     Resume mission (when paused or after override)",
                Style::default().fg(help_fg),
            )),
            Line::from(Span::styled(
                "  r     RTL (return to launch)",
                Style::default().fg(help_fg),
            )),
            Line::from(Span::styled("  l     Land", Style::default().fg(help_fg))),
            Line::from(Span::styled("  t     Takeoff 10 m", Style::default().fg(help_fg))),
            Line::from(Span::styled(
                "  s     Retry mission list + telemetry streams",
                Style::default().fg(help_fg),
            )),
            Line::from(""),
            Line::from(Span::styled(
                " If arm fails: try g then a, or use f for force arm. ",
                Style::default().fg(Color::DarkGray),
            )),
        ];
        let area = ratatui::layout::Rect {
            x: f.area().width.saturating_sub(52) / 2,
            y: f.area().height.saturating_sub(20) / 2,
            width: 52.min(f.area().width),
            height: 20.min(f.area().height),
        };
        f.render_widget(Clear, area);
        let block = Block::default()
            .title(" Help ")
            .borders(Borders::ALL)
            .border_style(Style::default().fg(Color::DarkGray))
            .style(Style::default().bg(help_bg));
        let inner = block.inner(area);
        f.render_widget(block, area);
        f.render_widget(
            Paragraph::new(help_text)
                .wrap(Wrap { trim: true })
                .style(Style::default().bg(help_bg).fg(help_fg)),
            inner,
        );
    }
}
