//! Runs commands and monitors their completion.

use std::{env, process::Stdio};

use color_eyre::eyre::{self, eyre, WrapErr};
use command_group::{AsyncCommandGroup, AsyncGroupChild};
use nix::unistd::Pid;
use once_cell::sync::Lazy;
use regex::{Captures, Regex};
use tokio::{
    io::{AsyncBufReadExt, BufReader},
    sync::oneshot,
};

use crate::config::CommandConfig;

/// Exit status returned by a command.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub(crate) enum ExitStatus {
    /// Command exited with the given exit code.
    Exited(i32),

    /// Command was killed before it could exit.
    Killed,
}

/// Control handle for a Command, used to send signals to the command.
#[derive(Debug)]
pub(crate) struct CommandControl {
    name: String,
    pid: Pid,
}

impl CommandControl {
    /// Sends a signal to the process.
    pub(crate) fn kill(&self, signal: nix::sys::signal::Signal) -> eyre::Result<()> {
        nix::sys::signal::kill(self.pid, signal).wrap_err_with(|| {
            format!("Error sending {signal} signal to process \"{}\"", self.name)
        })?;
        Ok(())
    }
}

/// Monitoring handle for a Command, used to wait for the Command to
/// exit.
#[derive(Debug)]
pub(crate) struct CommandMonitor {
    monitor: oneshot::Receiver<ExitStatus>,
}

impl CommandMonitor {
    /// Waits for the command to exit and returns the exit status.
    pub(crate) async fn wait(self) -> ExitStatus {
        self.monitor
            .await
            .expect("Command Monitor sender dropped before sending a result.")
    }
}

/// Runs the command and returns the control and monitor handles.
pub(crate) fn run(
    name: &str,
    config: &CommandConfig,
) -> eyre::Result<(CommandControl, CommandMonitor)> {
    tracing::debug!(%name, ?config, "Running command");

    // Initialize the command.
    let mut command = tokio::process::Command::new(&config.program);

    // Add the arguments, and perform environment variable substitution.
    match config
        .args
        .iter()
        .map(substitute_env_var)
        .collect::<eyre::Result<Vec<String>>>()
    {
        Ok(args) => command.args(args),
        Err(err) => {
            return Err(err.wrap_err(format!(
                "Environment variable expansion failed for command \"{}\"",
                config.program
            )))
        }
    };

    // Clear the environment if `only_env` was provided, then add back
    // in `PATH` and any other allowed environment variables.
    if let Some(only_env) = &config.only_env {
        command.env_clear();

        if let Ok(path) = env::var("PATH") {
            command.env("PATH", path);
        }

        for key in only_env {
            command.env(
                key,
                env::var(key).map_err(|_| eyre!("Unknown environment variable \"{key}\""))?,
            );
        }
    }

    // Set the uid and gid if provided.
    if let Some(username) = &config.user {
        let user = users::get_user_by_name(username)
            .ok_or_else(|| eyre!("Unknown username \"{username}\""))?;
        command.uid(user.uid()).gid(user.primary_group_id());
    };

    // Disable stdin, and pipe stdout and stderr so that we can read
    // and process the output.
    command
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());

    // Run the command.
    let mut child = command
        .group_spawn()
        .wrap_err_with(|| format!("Error starting command \"{}\"", config.program))?;
    let pid = nix::unistd::Pid::from_raw(child.id().ok_or_else(|| {
        eyre!(
            "Failed to get PID of just-started command \"{}\"",
            config.program
        )
    })? as i32);

    tracing::debug!(%name, %pid, "Command running");

    // Read stdout and stderr and send them to the console via
    // specially-targeted `tracing` events.
    let stdout = child
        .inner()
        .stdout
        .take()
        .expect("failed to get stdout from child process");
    let mut reader = BufReader::new(stdout).lines();
    let process = name.to_string();
    tokio::task::spawn({
        async move {
            while let Ok(Some(line)) = reader.next_line().await {
                tracing::info!(target: "stdout", %process, output = line);
            }
        }
    });

    let stderr = child
        .inner()
        .stderr
        .take()
        .expect("failed to get stderr from child process");
    let mut reader = BufReader::new(stderr).lines();
    let process = name.to_string();
    tokio::task::spawn({
        async move {
            while let Ok(Some(line)) = reader.next_line().await {
                tracing::info!(target: "stderr", %process, output = line);
            }
        }
    });

    // Listen for the command to complete.
    let (sender, receiver) = oneshot::channel();
    monitor_process(name.to_owned(), pid, child, sender);

    // Return the Command Control and Monitor.
    Ok((
        CommandControl {
            name: name.to_owned(),
            pid,
        },
        CommandMonitor { monitor: receiver },
    ))
}

fn substitute_env_var(s: impl AsRef<str>) -> eyre::Result<String> {
    static TEMPLATE_VAR_REGEX: Lazy<Regex> =
        Lazy::new(|| Regex::new(r"\{\{ *([A-Za-z0-9_]+) *\}\}").expect("regex should be valid"));

    // Make sure that every variable mentioned in a template expression
    // is a valid environment variable, returning an error if one or
    // more unknown variables are found. Otherwise replace all of the
    // template expressions with the value of the associated environment
    // variable.
    TEMPLATE_VAR_REGEX
        .captures_iter(s.as_ref())
        .map(|caps| {
            std::env::var(&caps[1])
                .map_err(|_| eyre!("Unknown environment variable \"{}\"", &caps[1]))
        })
        .collect::<eyre::Result<String>>()?;

    Ok(TEMPLATE_VAR_REGEX
        .replace_all(s.as_ref(), |caps: &Captures| {
            std::env::var(&caps[1]).expect("Unable to find environment variable")
        })
        .into_owned())
}

fn monitor_process(
    name: String,
    pid: Pid,
    mut child: AsyncGroupChild,
    sender: oneshot::Sender<ExitStatus>,
) {
    tokio::spawn(async move {
        match child.wait().await {
            Err(err) => {
                tracing::error!(%name, ?err, "Error waiting for command to exit");
                let _ = sender.send(ExitStatus::Killed);
            }
            Ok(exit_status) => match exit_status.code() {
                Some(exit_code) => {
                    if exit_code == 0 {
                        tracing::debug!(%name, %pid, "Command exited cleanly");
                    } else {
                        tracing::error!(%name, %pid, %exit_code, "Command exited with non-zero exit code");
                    }

                    let _ = sender.send(ExitStatus::Exited(exit_code));
                }
                None => {
                    tracing::debug!(%name, %pid, "Command was killed");
                    let _ = sender.send(ExitStatus::Killed);
                }
            },
        }
    });
}
