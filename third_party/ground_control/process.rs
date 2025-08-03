//! Starts and stops processes.

use color_eyre::eyre::{self, eyre, WrapErr};
use tokio::sync::{mpsc, oneshot};

use crate::{
    command::{self, CommandControl, ExitStatus},
    config::{CommandConfig, ProcessConfig, StopMechanism},
    ShutdownReason,
};

/// Process being managed by Ground Control.
#[derive(Debug)]
pub(crate) struct Process {
    config: ProcessConfig,
    handle: ProcessHandle,
}

#[derive(Debug)]
enum ProcessHandle {
    Daemon(CommandControl, oneshot::Receiver<ExitStatus>),
    OneShot,
}

/// Starts the process and returns a handle to the process.
pub(crate) async fn start_process(
    config: ProcessConfig,
    process_stopped: mpsc::UnboundedSender<ShutdownReason>,
) -> eyre::Result<Process> {
    tracing::info!("Starting process {}", config.name);

    // Perform the pre-run action, if provided.
    if let Some(pre_run) = &config.pre {
        run_process_command(&config.name, ProcessPhase::PreRun, pre_run).await?;
    }

    // Run the process itself (if this is a daemon process with a `run`
    // command).
    let handle = if let Some(run) = &config.run {
        let (daemon_sender, daemon_receiver) = oneshot::channel();

        let (control, monitor) = command::run(&config.name, run)
            .wrap_err_with(|| format!("`run` command failed for process \"{}\"", config.name))?;

        // Spawn a task to wait for the command to exit, then notify
        // both ourselves (to allow `stop` to return) and the shutdown
        // listener that our daemon process has exited.
        let process_name = config.name.clone();
        tokio::spawn(async move {
            let exit_status = monitor.wait().await;

            // TODO: Should this ever really happen? I would prefer to
            // just `expect` here if it is not possible. *But,* we need
            // to verify that, during some sort of startup/shutdown
            // failure, that we do not drop things too early and then
            // the receiver is gone.
            if daemon_sender.send(exit_status).is_err() {
                tracing::error!(process = %process_name, "Daemon receiver dropped before receiving exit signal.");
            }

            let shutdown_reason = match exit_status {
                ExitStatus::Exited(0) => ShutdownReason::DaemonExited,
                ExitStatus::Exited(_) | ExitStatus::Killed => ShutdownReason::DaemonFailed,
            };

            if let Err(err) = process_stopped.send(shutdown_reason) {
                tracing::error!(
                    process = %process_name,
                    ?err,
                    "Shutdown receiver dropped before all processes have exited."
                );
            }
        });

        ProcessHandle::Daemon(control, daemon_receiver)
    } else {
        ProcessHandle::OneShot
    };

    Ok(Process { config, handle })
}

impl Process {
    /// Stops the process: executes the `stop` command/signal if this is
    /// a daemon process; waits for the process to exit; runs the `post`
    /// command (if present).
    pub(crate) async fn stop_process(self) -> eyre::Result<()> {
        tracing::info!("Stopping process {}", self.config.name);

        // Stop the process (which is only required for daemon
        // processes; one-shot processes never "started").
        match self.handle {
            ProcessHandle::Daemon(control, mut daemon_receiver) => {
                // Has the daemon already shut down? If so, we do not
                // need to stop it (we just need to run the `post`
                // command, if any). Note that, if the `stop` operation
                // fails, we will *not* wait for the daemon to exit,
                // since it probably did not get our stop signal.
                if daemon_receiver.try_recv().is_ok() {
                    tracing::debug!(process = %self.config.name, "Process already exited; no need to `stop` it.");
                } else if let Err(err) = match self.config.stop {
                    StopMechanism::Signal(signal) => control.kill(signal.into()),
                    StopMechanism::Command(command) => {
                        run_process_command(&self.config.name, ProcessPhase::Stop, &command).await
                    }
                } {
                    tracing::warn!(process = %self.config.name, ?err, "Error stopping process.");
                } else {
                    // Wait for the daemon to stop.
                    match daemon_receiver.await {
                        Ok(ExitStatus::Exited(0)) => {
                            tracing::debug!(process = %self.config.name, "Process exited cleanly");
                        }
                        Ok(ExitStatus::Exited(exit_code)) => {
                            tracing::warn!(process = %self.config.name, %exit_code, "Process exited with non-zero exit code");
                        }
                        Ok(ExitStatus::Killed) => {
                            tracing::warn!(process = %self.config.name, "Process was killed");
                        }
                        Err(_) => {
                            // TODO: Should this ever really happen? I
                            // would prefer to just `expect` here if it
                            // is not possible. *But,* we need to verify
                            // that, during some sort of
                            // startup/shutdown failure, that we do not
                            // drop things too early and then receiver
                            // is gone.
                            tracing::error!("Daemon sender dropped before delivering exit signal.")
                        }
                    }
                }
            }
            ProcessHandle::OneShot => {}
        };

        // Execute the `post`(-run) command.
        if let Some(post_run) = &self.config.post {
            run_process_command(&self.config.name, ProcessPhase::PostRun, post_run).await?;
        }

        // The process has been stopped.
        Ok(())
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum ProcessPhase {
    PreRun,
    Stop,
    PostRun,
}

impl std::fmt::Display for ProcessPhase {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ProcessPhase::PreRun => write!(f, "pre"),
            ProcessPhase::Stop => write!(f, "stop"),
            ProcessPhase::PostRun => write!(f, "post"),
        }
    }
}

/// Runs one of a process's "phase" commands -- `pre`, `stop`, or
/// `post`, but crucially, not `run` -- and returns the success or
/// failure of the command.
async fn run_process_command(
    process_name: &str,
    process_phase: ProcessPhase,
    command: &CommandConfig,
) -> eyre::Result<()> {
    let (_control, monitor) = command::run(&format!("{process_name}[{process_phase}]"), command)
        .wrap_err_with(|| {
            format!("`{process_phase}` command failed for process \"{process_name}\"")
        })?;

    match monitor.wait().await {
        ExitStatus::Exited(0) => Ok(()),
        ExitStatus::Exited(exit_code) => {
            Err(eyre!(
                "`{process_phase}` command failed for process \"{process_name}\" (exit code {exit_code})",
            ))
        }
        ExitStatus::Killed => {
            Err(eyre!(
                "`{process_phase}` command was killed for process \"{process_name}\"",
            ))
        }
    }
}
