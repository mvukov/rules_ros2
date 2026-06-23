//! Process manager designed for container-*like* environments that need
//! to run multiple processes, with basic dependency relationships and
//! pre/post execution commands.

#![forbid(unsafe_code, future_incompatible)]
#![deny(
    missing_debug_implementations,
    nonstandard_style,
    missing_docs,
    unreachable_pub,
    missing_copy_implementations,
    unused_qualifications,
    clippy::unwrap_in_result,
    clippy::unwrap_used
)]

use std::path::PathBuf;

use clap::Parser;
use color_eyre::eyre::{self, WrapErr};
use groundcontrol::config::Config;
use tokio::{
    signal::unix::{signal, SignalKind},
    sync::mpsc,
};

#[derive(Parser)]
#[clap(about, long_about = None)]
struct Cli {
    /// Check the configuration file for errors, but do not start any
    /// processes.
    #[clap(long)]
    check: bool,

    config_file: String,
}

/// Gets the ROS_HOME directory path.
///
/// Returns ROS_HOME if set, otherwise falls back to ~/.ros
fn get_ros_home() -> eyre::Result<PathBuf> {
    if let Ok(ros_home) = std::env::var("ROS_HOME") {
        Ok(PathBuf::from(ros_home))
    } else {
        let home = std::env::var("HOME").wrap_err("HOME environment variable not set")?;
        Ok(PathBuf::from(home).join(".ros"))
    }
}

/// Gets the log directory following ROS 2 strategy.
///
/// If ROS_LOG_DIR is defined: validate and use it directly, or fail if invalid.
/// If ROS_LOG_DIR is not defined: use ros_home/log and create a unique subdirectory.
fn get_log_dir(ros_home: &PathBuf) -> eyre::Result<PathBuf> {
    let base_log_dir = if let Ok(ros_log_dir) = std::env::var("ROS_LOG_DIR") {
        // ROS_LOG_DIR is explicitly set - validate it or fail
        if ros_log_dir.is_empty() {
            return Err(eyre::eyre!(
                "ROS_LOG_DIR is set but empty. Either unset it or provide a valid path."
            ));
        }

        PathBuf::from(&ros_log_dir)
    } else {
        ros_home.join("log")
    };
    make_unique_log_dir(&base_log_dir)
}

/// Gets the launch log file path within the given log directory.
fn get_launch_log_path(log_dir: &PathBuf) -> PathBuf {
    log_dir.join("launch.log")
}

/// Creates a unique log directory with format: YYYY-MM-DD-HH-MM-SS-micros-hostname-pid
fn make_unique_log_dir(base_path: &PathBuf) -> eyre::Result<PathBuf> {
    // Get current UTC time with microseconds
    let now = chrono::Utc::now();
    let datetime_str = format!(
        "{}-{:06}",
        now.format("%Y-%m-%d-%H-%M-%S"),
        now.timestamp_subsec_micros()
    );

    // Get hostname
    let hostname = hostname::get()
        .wrap_err("Failed to get hostname")?
        .to_string_lossy()
        .to_string();

    // Get PID
    let pid = std::process::id();

    // Construct directory name and path
    let log_dirname = format!("{}-{}-{}", datetime_str, hostname, pid);
    let log_dir = base_path.join(log_dirname);

    // Create the directory (will fail if it already exists or can't be created)
    std::fs::create_dir_all(&log_dir)
        .wrap_err_with(|| format!("Failed to create log directory: {}", log_dir.display()))?;

    Ok(log_dir)
}

#[tokio::main]
async fn main() -> eyre::Result<()> {
    // Install color-eyre hooks.
    color_eyre::install()?;

    // Crash the process on a panic anywhere (including in a background
    // Tokio task, since we want panic to mean "something is very wrong;
    // stop everything").
    std::panic::set_hook(Box::new(|info| {
        eprintln!("Process panicked: {info}");
        std::process::abort();
    }));

    // Parse the command line arguments.
    let cli = Cli::parse();

    // Read and parse the config file.
    let config_file = tokio::fs::read_to_string(cli.config_file)
        .await
        .wrap_err("Failed to read config file")?;
    let config: Config = toml::from_str(&config_file).wrap_err("Failed to parse config file")?;

    // We're done if this was only a config file check.
    if cli.check {
        return Ok(());
    }

    // Initialize the tracing subscriber with our custom formatter.
    // Default to INFO-level logging, but allow that to be overridden
    // using an environment variable.
    if std::env::var_os("RUST_LOG").is_none() {
        std::env::set_var("RUST_LOG", "info")
    }

    // Get the launch log file path following ROS 2 strategy
    let ros_home = get_ros_home().wrap_err("Failed to determine ROS_HOME")?;
    let log_dir = get_log_dir(&ros_home).wrap_err("Failed to determine log directory")?;

    // Export ROS_HOME and ROS_LOG_DIR as environment variables for child processes
    std::env::set_var("ROS_HOME", &ros_home);
    std::env::set_var("ROS_LOG_DIR", &log_dir);

    // Create a file appender for the launch log
    let launch_log_path = get_launch_log_path(&log_dir);
    let file = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(&launch_log_path)
        .wrap_err_with(|| {
            format!(
                "Failed to open launch log file: {}",
                launch_log_path.display()
            )
        })?;
    let (non_blocking_file, _guard) = tracing_appender::non_blocking(file);

    // Build the layers for console and file logging
    use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt};

    let env_filter = tracing_subscriber::EnvFilter::from_default_env();
    let formatter = groundcontrol::formatter::GroundControlFormatter::from_config(&config)
        .with_include_timestamp(!config.suppress_timestamps);

    // Console layer with custom formatter
    let console_layer = tracing_subscriber::fmt::layer()
        .event_format(formatter.clone())
        .with_writer(std::io::stdout);

    // File layer with custom formatter
    let file_layer = tracing_subscriber::fmt::layer()
        .event_format(formatter)
        .with_writer(non_blocking_file)
        .with_ansi(false); // Disable ANSI colors in file output

    // Initialize the subscriber with both layers
    tracing_subscriber::registry()
        .with(env_filter)
        .with(console_layer)
        .with(file_layer)
        .init();

    // Now that logging is initialized, log the directory location
    tracing::info!("All log files can be found below {}", log_dir.display());

    // Create the external shutdown signal (used to shut down Ground
    // Control on UNIX signals).
    let (shutdown_sender, mut shutdown_receiver) = mpsc::unbounded_channel();

    let sigint_shutdown_sender = shutdown_sender.clone();
    tokio::spawn(async move {
        signal(SignalKind::interrupt())
            .expect("Failed to register SIGINT handler")
            .recv()
            .await;
        let _ = sigint_shutdown_sender.send(());
    });

    let sigterm_shutdown_sender = shutdown_sender.clone();
    tokio::spawn(async move {
        signal(SignalKind::terminate())
            .expect("Failed to register SIGTERM handler")
            .recv()
            .await;
        let _ = sigterm_shutdown_sender.send(());
    });

    // Run the Ground Control specification, *unless* we are in
    // break-glass mode, in which case we freeze startup and just wait
    // for the shutdown signal. (this gives the admin a chance to SSH
    // into a machine that is in a startup-crash loop, perhaps due to an
    // issue on an attached, persistent storage volume)
    if std::env::var_os("BREAK_GLASS").is_none() {
        groundcontrol::run(config, shutdown_receiver).await?;
    } else {
        tracing::info!("BREAK GLASS MODE: no processes will be started");

        shutdown_receiver
            .recv()
            .await
            .expect("All shutdown senders closed without sending a shutdown signal.");

        tracing::info!(
            "Shutdown signal triggered (make sure to clear the `BREAK_GLASS` environment variable)"
        );
    }

    Ok(())
}
