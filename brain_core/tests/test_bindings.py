"""The bindings of a brain's generated package, found by where every
brain's peppygen puts each module."""

import sys
from dataclasses import fields

import pytest

from brain_core.bindings import GENERATED_MODULES, Bindings


def write_package(root, name: str, leave_out: str = "") -> None:
    """A generated package of the brain's shape under `root`: a module at
    each place GENERATED_MODULES names, but `leave_out`, and a QoSProfile."""
    package = root / name
    package.mkdir()
    (package / "__init__.py").write_text("class QoSProfile:\n    Standard = 'standard qos'\n")
    for path in GENERATED_MODULES.values():
        if path == leave_out:
            continue
        *folders, module = path.split(".")
        directory = package
        for folder in folders:
            directory = directory / folder
            directory.mkdir(exist_ok=True)
            (directory / "__init__.py").touch()
        (directory / f"{module}.py").write_text(f"WHERE = {path!r}\n")


@pytest.fixture
def importable(tmp_path, monkeypatch):
    monkeypatch.syspath_prepend(str(tmp_path))
    yield tmp_path
    for module in [m for m in sys.modules if m.split(".")[0] in ("fakegen", "partgen")]:
        del sys.modules[module]


def test_every_field_but_the_qos_comes_from_the_generated_package(importable):
    import importlib

    write_package(importable, "fakegen")
    bindings = Bindings.from_generated(importlib.import_module("fakegen"))
    assert set(GENERATED_MODULES) == {field.name for field in fields(Bindings)} - {"standard_qos"}
    assert bindings.standard_qos == "standard qos"
    for field, path in GENERATED_MODULES.items():
        module = getattr(bindings, field)
        assert module.__name__ == f"fakegen.{path}" and module.WHERE == path


def test_a_module_the_package_lacks_is_named(importable):
    import importlib

    write_package(importable, "partgen", leave_out="exposed_services.brain_model.list_models")
    with pytest.raises(ModuleNotFoundError, match="partgen.exposed_services.brain_model.list_models"):
        Bindings.from_generated(importlib.import_module("partgen"))
