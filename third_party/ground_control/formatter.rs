//! Custom formatter for use with Ground Control.

use std::{collections::HashMap, fmt::Write};

use console::{style, Style};
use time::macros::format_description;
use tracing::{
    field::{Field, Visit},
    Event, Level, Subscriber,
};
use tracing_subscriber::{
    fmt::{format::Writer, FmtContext, FormatEvent, FormatFields},
    registry::LookupSpan,
};

use crate::config::Config;

/// Formats tracing events using a columnar format.
#[derive(Clone, Debug)]
pub struct GroundControlFormatter {
    /// Whether or not to include the timestamp.
    include_timestamp: bool,

    /// Style to use for the Ground Control process.
    groundcontrol_style: Style,

    /// Style for one-shot processes.
    oneshot_style: Style,

    /// Console style to use for daemon processes (and their phases).
    daemon_styles: HashMap<String, Style>,

    /// Style to use for error strings.
    error_style: Style,
}

impl GroundControlFormatter {
    /// Create a GroundControlFormatter given a Ground Control config
    /// (which will be used to assign colors to all of the daemon
    /// processes).
    pub fn from_config(config: &Config) -> Self {
        // Assign a style to every phase of every daemon process.
        let styles = vec![
            Style::new().green().bold(),
            Style::new().blue().bold(),
            Style::new().yellow().bold(),
            Style::new().magenta().bold(),
            Style::new().cyan().bold(),
        ];
        let mut styles = styles.iter().cycle();

        let mut daemon_styles: HashMap<String, Style> = Default::default();
        for process in config.processes.iter().filter(|p| p.run.is_some()) {
            // Get the next style from the iterator.
            let style = styles
                .next()
                .expect("iterator should be infinite in length");

            // Add styles for all four process phases of this process to
            // the list.
            daemon_styles.extend([
                (format!("{}[pre]", process.name), style.clone()),
                (process.name.to_string(), style.clone()),
                (format!("{}[stop]", process.name), style.clone()),
                (format!("{}[post]", process.name), style.clone()),
            ]);
        }

        // Build and return the formatter.
        Self {
            include_timestamp: true,
            groundcontrol_style: Style::new().white().dim(),
            oneshot_style: Style::new().bold(),
            daemon_styles,
            error_style: Style::new().red().bold(),
        }
    }

    /// Whether or not to include the timestamp in the output.
    pub fn with_include_timestamp(mut self, include_timestamp: bool) -> Self {
        self.include_timestamp = include_timestamp;
        self
    }
}

impl<S, N> FormatEvent<S, N> for GroundControlFormatter
where
    S: Subscriber + for<'a> LookupSpan<'a>,
    N: for<'a> FormatFields<'a> + 'static,
{
    fn format_event(
        &self,
        _ctx: &FmtContext<'_, S, N>,
        mut writer: Writer<'_>,
        event: &Event<'_>,
    ) -> core::fmt::Result {
        // Generate the timestamp for this event.
        let format = format_description!(
            "[year]-[month]-[day]T[hour]:[minute]:[second].[subsecond digits:3]Z "
        );
        let timestamp = if self.include_timestamp {
            time::OffsetDateTime::now_utc()
                .format(&format)
                .map_err(|_| std::fmt::Error)?
        } else {
            String::new()
        };

        // Events that target "stdout" or "stderr" are from external
        // processes; everything else is from Ground Control.
        if event.metadata().target() == "stdout" || event.metadata().target() == "stderr" {
            let mut visitor: ConsoleOutputVisitor = Default::default();
            event.record(&mut visitor);

            let styled_process = self
                .daemon_styles
                .get(&visitor.process)
                .unwrap_or(&self.oneshot_style)
                .apply_to(visitor.process);

            writeln!(
                writer,
                "{}{}:{}{}",
                self.groundcontrol_style.apply_to(timestamp),
                styled_process,
                visitor.message,
                style(visitor.fields).white().dim()
            )
        } else {
            let mut visitor: EventVisitor = Default::default();
            event.record(&mut visitor);

            writeln!(
                writer,
                "{}{}:{}{}",
                self.groundcontrol_style.apply_to(timestamp),
                self.groundcontrol_style.apply_to("groundcontrol"),
                if *event.metadata().level() <= Level::WARN {
                    self.error_style.apply_to(visitor.message).to_string()
                } else {
                    visitor.message
                },
                style(visitor.fields).white().dim()
            )
        }
    }
}

#[derive(Clone, Debug, Default)]
struct EventVisitor {
    message: String,
    fields: String,
}

impl Visit for EventVisitor {
    fn record_str(&mut self, field: &Field, value: &str) {
        write!(self.fields, " {}={}", field.name(), value).unwrap();
    }

    fn record_debug(&mut self, field: &Field, value: &dyn std::fmt::Debug) {
        match field.name() {
            "message" => self.message = format!(" {value:?}"),
            _ => write!(self.fields, " {}={:?}", field.name(), value).unwrap(),
        }
    }
}

#[derive(Clone, Debug, Default)]
struct ConsoleOutputVisitor {
    process: String,
    message: String,
    fields: String,
}

impl Visit for ConsoleOutputVisitor {
    fn record_str(&mut self, field: &Field, value: &str) {
        match field.name() {
            "process" => self.process = value.to_string(),
            "output" => self.message = format!(" {value}"),
            _ => write!(self.fields, " {}={}", field.name(), value).unwrap(),
        }
    }

    fn record_debug(&mut self, field: &Field, value: &dyn std::fmt::Debug) {
        match field.name() {
            "message" => self.message = format!(" {value:?}"),
            _ => write!(self.fields, " {}={:?}", field.name(), value).unwrap(),
        }
    }
}
