# Contributing to rules_ros2

Thank you for your interest in contributing! The following guidelines will help
you get your changes merged smoothly.

## Getting started

1. **Fork** the repository and clone your fork locally.
2. Open the repo in the provided dev container
   (`.devcontainer/ubuntu_22_04_x86/`) — it comes with all required tooling
   pre-installed (Bazel, C++ compiler, Python 3.10, Rust, Docker socket, and
   the right shared-memory settings). In VS Code, choose
   **Dev Containers: Reopen in Container** from the command palette.

   If you prefer not to use the dev container, make sure you have the
   prerequisites described in the [README](README.md) installed manually.

## Pre-commit (required)

All contributions must pass the pre-commit checks before review. The hooks
enforce formatting and linting for Bazel files, C++, Python, Rust, and
Markdown.

### Install pre-commit

... only if you're **not** using the provided devcontainer:

```bash
pip install pre-commit
```

### Install the hooks

Run this once from the repo root:

```bash
pre-commit install
```

After that, the hooks run automatically on every `git commit`. To run them
manually against all files:

```bash
pre-commit run --all-files
```

The hooks include:

| Hook                             | What it checks                                                         |
| -------------------------------- | ---------------------------------------------------------------------- |
| `buildifier` / `buildifier-lint` | Bazel `BUILD` and `.bzl` files                                         |
| `yapf`                           | Python formatting                                                      |
| `reorder-python-imports`         | Python import ordering                                                 |
| `ruff`                           | Python linting                                                         |
| `clang-format`                   | C/C++ formatting                                                       |
| `cpplint`                        | C/C++ style                                                            |
| `prettier`                       | Markdown, YAML, and other files                                        |
| `rustfmt`                        | Rust formatting                                                        |
| Standard hooks                   | Large files, merge conflicts, YAML validity, trailing whitespace, etc. |

> **Note:** The `rustfmt` hook requires one-time setup: `./repositories/rust/rustfmt/generate.sh`.

## Making changes

- Keep pull requests focused on a single concern.
- Add or update tests under `ros2/test` or `examples/` directories when adding new
  functionality, so CI can verify the change end-to-end.
- Follow the existing code style — the pre-commit hooks will catch most
  deviations automatically.

## Submitting a pull request

1. Push your branch to your fork.
2. Open a pull request against the `main` branch.
3. Ensure all CI checks pass.
4. Respond to review feedback promptly.

## Reporting issues

Please open a GitHub issue with a clear description of the problem, the steps
to reproduce it, and the output of the failing command.
