# Benchmarks

## Domain ID isolation overhead

Every ROS 2 test target runs through a wrapper that reserves a unique
`ROS_DOMAIN_ID`, so that tests running in parallel don't talk to each other.
This benchmark measures how much time such a wrapper adds per test, comparing:

- **C++:** `//ros2/domain_coordinator:run_test_isolated`.
- **Python:** `:run_test_isolated_py`, an equivalent of upstream's
  `run_test_isolated.py` built on upstream's `domain_coordinator` package from
  `ament_cmake_ros` (`@ros2_ament_cmake_ros`). It's measured as a Bazel
  `py_binary`, i.e. how rules_ros2 would ship it, including the rules_python
  bootstrap.

Both wrap a no-op command (`/bin/true`), so any time beyond running `/bin/true`
directly is wrapper overhead. Each variant is measured in two scenarios:

- no other reservations, so the first domain ID tried is free;
- domain IDs 1-50 already held, to simulate many tests running in parallel, so
  the wrappers have to scan for a free ID.

### Running

```bash
bazel run -c opt //ros2/benchmarks:run_test_isolated_benchmark
```

Options:

- `--runs` (default 300) and `--warmup` (default 20): repetitions per variant.
- `--held-ids` (default 50): domain IDs held in the second scenario.
- `--pycache-prefix <dir>`: sets `PYTHONPYCACHEPREFIX` for the Python wrapper,
  to measure it with a warm bytecode cache (see below).

### Results

Measured on 2026-11-10 on a 12-core x86_64 Linux devcontainer, with Bazel
9.3.2, the hermetic Python 3.12.13 from rules_python, `-c opt` and 100 runs.
Numbers are the median time added on top of running `/bin/true` directly.

| Wrapper                                     | No other reservations | IDs 1-50 held |
| ------------------------------------------- | --------------------: | ------------: |
| C++ `run_test_isolated`                     |               +1.4 ms |       +1.5 ms |
| Python `py_binary`, cold bytecode cache     |               +209 ms |       +209 ms |
| Python `py_binary`, warm bytecode cache (1) |                +75 ms |        +77 ms |

(1) With `--pycache-prefix`.

Findings:

- **Finding a free domain ID is cheap.** Even with 50 IDs taken, the scan adds
  at most a few milliseconds, in either language.
- **The Python cost is startup, not the coordination itself.** The interpreter
  starts in ~16 ms, importing `socket` and `subprocess` adds ~25 ms, and the
  rules_python `py_binary` bootstrap adds roughly another 30 ms. That's ~75 ms
  in the best case.
- **Without a bytecode cache, Python is ~3x slower still.** The hermetic
  interpreter's stdlib comes without `.pyc` files. If Python can't write them,
  it recompiles `subprocess`, `socket` and their dependencies from source on
  every run, which costs ~210 ms per run.
- **For C++ and Rust unit tests, this overhead dominates.** A typical gtest
  finishes in a few milliseconds, so a Python wrapper would make up most of
  its run time. For launch tests, which start Python anyway, the difference
  matters much less. Across e.g. 500 test targets, a Python wrapper costs
  roughly 35-105 s of extra CPU time per full test run.

### Which Python number applies under `bazel test`?

rules_python points the hermetic stdlib's `__pycache__` directories to a
shared cache directory, `/tmp/rules_python_pycache/...` by default, or
`$XDG_CACHE_HOME/rules_python_pycache/...` if `XDG_CACHE_HOME` is set. That
directory is created only when the interpreter repository is fetched.

Measured with a test target that ran the Python wrapper 5 times per test run:

| Setup                                                                 | Python wrapper overhead per run                                                  |
| --------------------------------------------------------------------- | -------------------------------------------------------------------------------- |
| Cache directory missing (e.g. `/tmp` was cleared after the fetch)     | ~200 ms on every run, with any spawn strategy                                    |
| Cache directory present, `processwrapper-sandbox` or `local` strategy | ~200 ms on the very first run, then ~75 ms, also across `bazel test` invocations |

So the warm number is the best case. It applies only when every test can read
and write the shared cache. The cold number is likely what many setups get:

- With `linux-sandbox`, which this repo's `.bazelrc` recommends, the
  sandbox's private `/tmp` very likely hides the cache directory. rules_python's
  source suggests adding `--sandbox_add_mount_pair=/tmp/rules_python_pycache`
  for this case. This wasn't verified, since `linux-sandbox` isn't available in
  the devcontainer used for these measurements.
- CI runners start with a fresh `/tmp`, and restoring Bazel's repository cache
  doesn't recreate the cache directory. Setting `XDG_CACHE_HOME` to a persisted
  directory at fetch time may avoid this, but that wasn't verified either.

Either way, a Python wrapper adds at least ~75 ms per test, against ~1.4 ms for
the C++ one.
