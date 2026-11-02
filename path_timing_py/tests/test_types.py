"""The types of the module, path_timing_py.pyi, against the module itself."""

import subprocess
import sys
from pathlib import Path

PACKAGE = Path(__file__).parents[1]
STUBTEST_ALLOWLIST = Path(__file__).with_name("stubtest_allowlist.txt")


def test_the_types_name_and_sign_what_the_module_gives():
    # From the package directory, mypy reads the stub there: the one that
    # this repository holds.
    stubtest = subprocess.run(
        [
            sys.executable,
            "-m",
            "mypy.stubtest",
            "path_timing_py",
            "--allowlist",
            str(STUBTEST_ALLOWLIST),
        ],
        cwd=PACKAGE,
        capture_output=True,
        text=True,
        check=False,
    )
    assert stubtest.returncode == 0, stubtest.stdout + stubtest.stderr
