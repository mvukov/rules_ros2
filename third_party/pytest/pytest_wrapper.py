import os
import pathlib
import sys

import coverage
import pytest


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
    args = sys.argv[1:]

    bazel_coverage = os.getenv('COVERAGE') == '1'

    coverage_session = None
    if bazel_coverage:
        coverage_session = start_coverage_session()

    args.append(f'--junitxml={os.environ["XML_OUTPUT_FILE"]}')
    pytest_exit_code = pytest.main(args)

    if coverage_session:
        finalize_coverage_session(coverage_session)

    sys.exit(pytest_exit_code)


if __name__ == '__main__':
    main()
