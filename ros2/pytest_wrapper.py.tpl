# Adapted from https://gist.github.com/betaboon/c1dd785b5ba468b4df4e382eafff969a

import contextlib
import os
import pathlib
import sys

import coverage
import domain_coordinator
import pytest

{ament_setup}


def start_coverage_session() -> coverage.Coverage:
    coverage_dir = pathlib.Path(os.getenv('COVERAGE_DIR'))
    coverage_file = coverage_dir / '.coverage'
    coverage_manifest = pathlib.Path(os.getenv('COVERAGE_MANIFEST'))
    coverage_sources = coverage_manifest.read_text().splitlines()
    coverage_session = coverage.Coverage(
        data_file=str(coverage_file),
        include=coverage_sources,
    )
    coverage_session.start()
    return coverage_session


def finalize_coverage_session(coverage_session: coverage.Coverage) -> None:
    coverage_session.stop()
    coverage_session.save()
    output_file = pathlib.Path(os.getenv('COVERAGE_OUTPUT_FILE'))
    coverage_session.lcov_report(outfile=output_file)


def main() -> None:
    test_outputs_dir = os.environ.get('TEST_UNDECLARED_OUTPUTS_DIR') or os.environ.get('TEST_TMPDIR')
    if test_outputs_dir:
        os.environ['ROS_HOME'] = test_outputs_dir
        os.environ['ROS_LOG_DIR'] = test_outputs_dir

    with contextlib.ExitStack() as stack:
        if 'ROS_DOMAIN_ID' not in os.environ:
            domain_id = stack.enter_context(domain_coordinator.domain_id())
            os.environ['ROS_DOMAIN_ID'] = str(domain_id)
        print(f'Running with ROS_DOMAIN_ID {os.environ["ROS_DOMAIN_ID"]}')

        bazel_coverage = os.getenv('COVERAGE') == '1'

        coverage_session = None
        if bazel_coverage:
            coverage_session = start_coverage_session()

        args = [
            '-ra', '-vv', '-p', 'launch_pytest.plugin',
            f'--junitxml={os.environ["XML_OUTPUT_FILE"]}'
        ] + sys.argv[1:]
        pytest_exit_code = pytest.main(args)

        if coverage_session:
            finalize_coverage_session(coverage_session)

        sys.exit(pytest_exit_code)


if __name__ == '__main__':
    main()
