"""The bundled models from Python: each by its id, its files, and the files
written to disk for a reader that takes them from there."""

import pytest

import robot_description_py
from robot_description_py import model, models


def test_each_bundled_model_is_found_by_its_id():
    ids = [bundled.id for bundled in models()]
    assert ids == sorted(ids)
    assert {"openarm_v1", "openarm_v2", "so101"} <= set(ids)
    for bundled in models():
        assert model(bundled.id).id == bundled.id


def test_an_id_of_no_model_raises_naming_the_models():
    names = ", ".join(f'"{bundled.id}"' for bundled in models())
    with pytest.raises(ValueError) as refusal:
        model("openarm_v3")
    assert str(refusal.value) == f'no robot model is named "openarm_v3"; the models are {names}'


def test_a_model_gives_its_files_and_their_digest():
    so101 = model("so101")
    assert "<robot" in so101.urdf and "<robot" in so101.srdf
    assert len(so101.digest) == 64 and int(so101.digest, 16) >= 0
    names = [name for name, _ in so101.meshes]
    assert names == sorted(names) and names
    for name, data in so101.meshes:
        assert name in so101.urdf, f"{name} is a mesh the URDF names"
        assert isinstance(data, bytes) and data


def test_written_files_are_the_models_files(tmp_path):
    so101 = model("so101")
    so101.write_to(tmp_path / "so101")
    directory = tmp_path / "so101"
    assert (directory / "so101.urdf").read_text() == so101.urdf
    assert (directory / "so101.srdf").read_text() == so101.srdf
    written = {path.name: path.read_bytes() for path in (directory / "meshes").iterdir()}
    assert written == dict(so101.meshes)


def test_the_state_names_are_the_ones_the_srdf_gives():
    assert (
        robot_description_py.READY,
        robot_description_py.HOME,
        robot_description_py.START,
        robot_description_py.PARK,
    ) == ("ready", "home", "start", "park")
    assert (robot_description_py.CLOSED, robot_description_py.OPEN) == ("closed", "open")
