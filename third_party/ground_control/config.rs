//! Configuration structs.

use std::collections::{HashMap, HashSet};

use serde::Deserialize;

/// Ground Control configuration.
#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    /// Suppress the timestamp field from the log output (useful on
    /// systems that prepend the log output with their own, timestamped
    /// log output).
    #[serde(default)]
    pub suppress_timestamps: bool,

    /// Optional list of additional variables to add to the environment.
    #[serde(default)]
    pub env: HashMap<String, String>,

    /// *Ordered* list of processes to start.
    pub processes: Vec<ProcessConfig>,
}

/// Process configuration.
#[derive(Clone, Debug, Deserialize)]
#[serde(rename_all = "kebab-case", deny_unknown_fields)]
pub struct ProcessConfig {
    /// Name of the process (used in logging/monitoring).
    pub name: String,

    /// Optional command to run *before* the `run` command.
    #[serde(default)]
    pub pre: Option<CommandConfig>,

    /// Optional `run` command; if present, this process is considered a
    /// "daemon process" and Ground Control will monitor the run
    /// command, shutting down all of the processes if any run command
    /// exits.
    #[serde(default)]
    pub run: Option<CommandConfig>,

    /// Mechanism for stopping the process *if this is a daemon process*
    /// (ignored if the process does not have a `run` command).
    #[serde(default)]
    pub stop: StopMechanism,

    /// Optional command to run after the process has been stopped.
    #[serde(default)]
    pub post: Option<CommandConfig>,
}

/// Mechanism used to stop a daemon process.
#[derive(Clone, Eq, PartialEq, Debug, Deserialize)]
#[serde(untagged)]
pub enum StopMechanism {
    /// Stop the process using a signal.
    Signal(SignalConfig),

    /// Stop the process by running a command.
    Command(CommandConfig),
}

impl Default for StopMechanism {
    fn default() -> Self {
        StopMechanism::Signal(SignalConfig::SIGTERM)
    }
}

/// Signals used to stop a daemon process.
#[derive(Copy, Clone, Eq, PartialEq, Debug, Deserialize)]
pub enum SignalConfig {
    /// SIGINT
    SIGINT,

    /// SIGQUIT
    SIGQUIT,

    /// SIGTERM
    SIGTERM,
}

impl From<SignalConfig> for nix::sys::signal::Signal {
    fn from(signal: SignalConfig) -> Self {
        match signal {
            SignalConfig::SIGINT => Self::SIGINT,
            SignalConfig::SIGQUIT => Self::SIGQUIT,
            SignalConfig::SIGTERM => Self::SIGTERM,
        }
    }
}

impl From<&SignalConfig> for nix::sys::signal::Signal {
    fn from(signal: &SignalConfig) -> Self {
        match signal {
            SignalConfig::SIGINT => Self::SIGINT,
            SignalConfig::SIGQUIT => Self::SIGQUIT,
            SignalConfig::SIGTERM => Self::SIGTERM,
        }
    }
}

/// Configuration for a command, its arguments, and any execution
/// properties (such as the user under which to run the command, or the
/// environment variables to pass through to the command).
#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(from = "CommandLineConfig")]
pub struct CommandConfig {
    /// User to run this command as, otherwise run the command as the
    /// user that executed Ground Control (most likely `root`).
    pub user: Option<String>,

    /// If present, then only the given list of environment variables
    /// will be passed through to the command (all other variables will
    /// be removed from the command's environment). Note that `PATH` is
    /// always allowed. All environment variables will be allowed if
    /// this value is `None`. If provided, but empty, then no variables
    /// other than `PATH` will be allowed.
    pub only_env: Option<HashSet<String>>,

    /// Program to execute.
    pub program: String,

    /// Arguments to pass to the program.
    pub args: Vec<String>,
}

#[derive(Clone, Eq, PartialEq, Debug, Deserialize)]
#[serde(untagged)]
enum CommandLineConfig {
    Simple(CommandLine),

    Detailed(DetailedCommandLine),
}

impl From<CommandLineConfig> for CommandConfig {
    fn from(config: CommandLineConfig) -> Self {
        match config {
            CommandLineConfig::Simple(config) => {
                let (program, args) = config.program_and_args();
                Self {
                    user: None,
                    only_env: None,
                    program,
                    args,
                }
            }
            CommandLineConfig::Detailed(config) => {
                let (program, args) = config.command.program_and_args();
                Self {
                    user: config.user,
                    only_env: config.only_env,
                    program,
                    args,
                }
            }
        }
    }
}

#[derive(Clone, Eq, PartialEq, Debug, Deserialize)]
#[serde(untagged)]
enum CommandLine {
    CommandString(String),

    CommandVector(Vec<String>),
}

impl CommandLine {
    /// Parse the Command Line into the program to execute, and the
    /// arguments to that program.
    fn program_and_args(&self) -> (String, Vec<String>) {
        match self {
            CommandLine::CommandString(line) => {
                // TODO: This won't handle quoted arguments with spaces
                // (for example), so really we should parse this using a
                // more correct, shell-like parser. OTOH, we could just
                // say that anything complicated needs to use the vector
                // format...
                let mut elems = line.split(' ');

                let program = elems
                    .next()
                    .expect("Command line must not be empty")
                    .to_string();
                let args = elems.map(|s| s.to_string()).collect();

                (program, args)
            }

            CommandLine::CommandVector(v) => {
                let program = v[0].to_string();
                let args = v[1..].to_vec();

                (program, args)
            }
        }
    }
}

#[derive(Clone, Eq, PartialEq, Debug, Deserialize)]
#[serde(deny_unknown_fields, rename_all = "kebab-case")]
struct DetailedCommandLine {
    #[serde(default)]
    user: Option<String>,

    #[serde(default)]
    only_env: Option<HashSet<String>>,

    command: CommandLine,
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use serde::Deserialize;

    use super::*;

    #[derive(Debug, Deserialize, PartialEq)]
    struct StopMechanismTest {
        stop: StopMechanism,
    }

    #[test]
    fn supports_signal_names_in_stop() {
        let toml = r#"stop = "SIGTERM""#;
        let decoded: StopMechanismTest = toml::from_str(toml).expect("Failed to parse test TOML");
        assert_eq!(StopMechanism::Signal(SignalConfig::SIGTERM), decoded.stop);
    }

    #[derive(Debug, Deserialize, PartialEq)]
    struct CommandConfigTest {
        run: CommandConfig,
    }

    #[test]
    fn supports_whitespace_separated_command_lines() {
        let toml = r#"run = "/app/run-me.sh using these args""#;
        let decoded: CommandConfigTest = toml::from_str(toml).expect("Failed to parse test TOML");
        assert_eq!(
            CommandConfig {
                user: None,
                only_env: None,
                program: String::from("/app/run-me.sh"),
                args: vec![
                    String::from("using"),
                    String::from("these"),
                    String::from("args"),
                ]
            },
            decoded.run
        );
    }

    #[test]
    fn supports_command_vectors() {
        let toml = r#"run = ["/app/run-me.sh", "using", "these", "args"]"#;
        let decoded: CommandConfigTest = toml::from_str(toml).expect("Failed to parse test TOML");
        assert_eq!(
            CommandConfig {
                user: None,
                only_env: None,
                program: String::from("/app/run-me.sh"),
                args: vec![
                    String::from("using"),
                    String::from("these"),
                    String::from("args"),
                ]
            },
            decoded.run
        );
    }

    #[test]
    fn supports_detailed_whitespace_separated_command_lines() {
        let toml = r#"run = { command = "/app/run-me.sh using these args" }"#;
        let decoded: CommandConfigTest = toml::from_str(toml).expect("Failed to parse test TOML");
        assert_eq!(
            CommandConfig {
                user: None,
                only_env: None,
                program: String::from("/app/run-me.sh"),
                args: vec![
                    String::from("using"),
                    String::from("these"),
                    String::from("args"),
                ]
            },
            decoded.run
        );

        let toml = r#"run = { user = "app", command = "/app/run-me.sh using these args" }"#;
        let decoded: CommandConfigTest = toml::from_str(toml).expect("Failed to parse test TOML");
        assert_eq!(
            CommandConfig {
                user: Some(String::from("app")),
                only_env: None,
                program: String::from("/app/run-me.sh"),
                args: vec![
                    String::from("using"),
                    String::from("these"),
                    String::from("args"),
                ]
            },
            decoded.run
        );
    }

    #[test]
    fn supports_detailed_command_vectors() {
        let toml = r#"run = { command = ["/app/run-me.sh", "using", "these", "args"] }"#;
        let decoded: CommandConfigTest = toml::from_str(toml).expect("Failed to parse test TOML");
        assert_eq!(
            CommandConfig {
                user: None,
                only_env: None,
                program: String::from("/app/run-me.sh"),
                args: vec![
                    String::from("using"),
                    String::from("these"),
                    String::from("args"),
                ]
            },
            decoded.run
        );

        let toml = r#"run = { user = "app", only-env = [], command = ["/app/run-me.sh", "using", "these", "args"] }"#;
        let decoded: CommandConfigTest = toml::from_str(toml).expect("Failed to parse test TOML");
        assert_eq!(
            CommandConfig {
                user: Some(String::from("app")),
                only_env: Some(HashSet::new()),
                program: String::from("/app/run-me.sh"),
                args: vec![
                    String::from("using"),
                    String::from("these"),
                    String::from("args"),
                ]
            },
            decoded.run
        );

        let toml = r#"run = { user = "app", only-env = ["USER", "HOME"], command = ["/app/run-me.sh", "using", "these", "args"] }"#;
        let decoded: CommandConfigTest = toml::from_str(toml).expect("Failed to parse test TOML");
        assert_eq!(
            CommandConfig {
                user: Some(String::from("app")),
                only_env: Some(HashSet::from(["USER".into(), "HOME".into()])),
                program: String::from("/app/run-me.sh"),
                args: vec![
                    String::from("using"),
                    String::from("these"),
                    String::from("args"),
                ]
            },
            decoded.run
        );
    }

    #[test]
    fn requires_command_in_detailed_command() {
        let toml = r#"run = { }"#;
        let error = toml::from_str::<CommandConfigTest>(toml).unwrap_err();
        assert_eq!("data did not match any variant of untagged enum CommandLineConfig for key `run` at line 1 column 1", error.to_string(),);

        let toml = r#"run = { user = "app" }"#;
        let error = toml::from_str::<CommandConfigTest>(toml).unwrap_err();
        assert_eq!("data did not match any variant of untagged enum CommandLineConfig for key `run` at line 1 column 1", error.to_string(),);
    }
}
