"""Benchmarks the overhead of ROS domain ID isolation wrappers.

Each wrapper runs a no-op command (/bin/true), so all extra time compared to
running /bin/true directly is the wrapper's overhead: process startup, domain
ID reservation and spawning the command.

Two scenarios are measured:
- no other reservations: the first domain ID tried is free;
- with --held-ids N (default 50) domain IDs already held, to simulate tests
  running in parallel, so the wrappers have to scan for a free ID.

The Python wrapper is measured as a Bazel py_binary, i.e. how it'd be used by
rules_ros2, including the rules_python bootstrap. Its startup time heavily
depends on whether Python can write bytecode caches for the hermetic
interpreter's stdlib. Pass --pycache-prefix to measure with a warm cache.
"""
import argparse
import os
import socket
import statistics
import subprocess
import time

# Must match domain_coordinator's port base.
PORT_BASE = 32768
NO_OP_COMMAND = '/bin/true'


def time_command(command, env, runs, warmup):
    for _ in range(warmup):
        subprocess.run(command, env=env, check=True)
    durations_ms = []
    for _ in range(runs):
        start = time.perf_counter()
        subprocess.run(command, env=env, check=True)
        durations_ms.append((time.perf_counter() - start) * 1e3)
    durations_ms.sort()
    return {
        'median': statistics.median(durations_ms),
        'p90': durations_ms[int(0.9 * (runs - 1))],
        'min': durations_ms[0],
    }


def run_scenario(title, variants, env, runs, warmup):
    print(f'\n{title} (runs={runs}, times in ms)')
    print(f'{"variant":24s} {"median":>8s} {"p90":>8s} {"min":>8s} '
          f'{"overhead":>9s}')
    baseline = None
    for name, command in variants.items():
        stats = time_command(command, env, runs, warmup)
        if baseline is None:
            baseline = stats['median']
        print(f'{name:24s} {stats["median"]:8.2f} {stats["p90"]:8.2f} '
              f'{stats["min"]:8.2f} {stats["median"] - baseline:+9.2f}')


def hold_domain_ids(count):
    sockets = []
    for domain_id in range(1, count + 1):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind(('', PORT_BASE + domain_id))
        sockets.append(s)
    return sockets


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', required=True, help='C++ wrapper path.')
    parser.add_argument('--py', required=True, help='Python wrapper path.')
    parser.add_argument('--runs', type=int, default=300)
    parser.add_argument('--warmup', type=int, default=20)
    parser.add_argument('--held-ids', type=int, default=50)
    parser.add_argument(
        '--pycache-prefix',
        help='If set, passed as PYTHONPYCACHEPREFIX to the Python wrapper.')
    args = parser.parse_args()

    env = {
        k: v
        for k, v in os.environ.items()
        if k not in ('ROS_DOMAIN_ID', 'DISABLE_ROS_ISOLATION')
    }
    if args.pycache_prefix:
        env['PYTHONPYCACHEPREFIX'] = os.path.abspath(args.pycache_prefix)

    variants = {
        f'baseline {NO_OP_COMMAND}': [NO_OP_COMMAND],
        'C++ run_test_isolated': [os.path.abspath(args.cc), NO_OP_COMMAND],
        'Python py_binary': [os.path.abspath(args.py), NO_OP_COMMAND],
    }

    run_scenario('No other reservations', variants, env, args.runs, args.warmup)
    held = hold_domain_ids(args.held_ids)
    try:
        run_scenario(f'Domain IDs 1-{args.held_ids} already held', variants,
                     env, args.runs, args.warmup)
    finally:
        for s in held:
            s.close()


if __name__ == '__main__':
    main()
