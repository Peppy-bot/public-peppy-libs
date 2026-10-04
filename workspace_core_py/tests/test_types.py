"""The types of the module, workspace_core_py.pyi, against the module itself."""

import ast
import subprocess
import sys
from pathlib import Path

import workspace_core_py

PACKAGE = Path(__file__).parents[1]
STUB = PACKAGE / "workspace_core_py.pyi"
STUBTEST_ALLOWLIST = Path(__file__).with_name("stubtest_allowlist.txt")


def test_the_types_name_and_sign_what_the_module_gives():
    # From the package directory, mypy reads the stub there: the one that
    # this repository holds.
    stubtest = subprocess.run(
        [
            sys.executable,
            "-m",
            "mypy.stubtest",
            "workspace_core_py",
            "--allowlist",
            str(STUBTEST_ALLOWLIST),
        ],
        cwd=PACKAGE,
        capture_output=True,
        text=True,
        check=False,
    )
    assert stubtest.returncode == 0, stubtest.stdout + stubtest.stderr


def sets_hash_to_none(statement: ast.stmt) -> bool:
    """Whether `statement`, of a class body, types `__hash__` as None: the
    class does not hash."""
    return (
        isinstance(statement, ast.AnnAssign)
        and isinstance(statement.target, ast.Name)
        and statement.target.id == "__hash__"
        and ast.unparse(statement.annotation) == "ClassVar[None]"
    )


def test_the_types_say_which_classes_hash_as_the_module_does():
    # stubtest does not compare __hash__. A class that compares by value
    # without a hash does not hash; another class hashes.
    classes = [
        node
        for node in ast.parse(STUB.read_text()).body
        if isinstance(node, ast.ClassDef)
    ]
    typed = {
        node.name: not any(sets_hash_to_none(statement) for statement in node.body)
        for node in classes
    }
    runtime = {
        name: getattr(workspace_core_py, name).__hash__ is not None for name in typed
    }
    assert typed == runtime
    assert not typed["Reach"] and not typed["GraspDirection"]
