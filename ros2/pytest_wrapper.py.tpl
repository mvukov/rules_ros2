# Adapted from https://gist.github.com/betaboon/c1dd785b5ba468b4df4e382eafff969a

import os
import pathlib
import sys

import coverage
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
    bazel_test_output_dir = os.environ.get('TEST_UNDECLARED_OUTPUTS_DIR')
    if bazel_test_output_dir is None:
        bazel_test_output_dir = os.environ.get('TEST_TMPDIR')
    if 'ROS_HOME' not in os.environ:
        os.environ['ROS_HOME'] = bazel_test_output_dir
    if 'ROS_LOG_DIR' not in os.environ:
        os.environ['ROS_LOG_DIR'] = bazel_test_output_dir

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
