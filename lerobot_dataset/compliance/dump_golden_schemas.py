"""Generate golden fixtures from a dataset written by Python lerobot itself.

Writes, under --out:
  manifest.json                    lerobot version + generation parameters
  info.json / stats.json           copied from the generated dataset's meta/
  schema.<name>.json               arrow schema + key-value metadata for every
                                   parquet the library produced
  video.<camera>.json              ffprobe stream facts for one video file

Rust tests assert the crate's output matches these field-for-field, so list
vs fixed-size-list choices, pandas index metadata, and episode-metadata
columns are pinned to lerobot ground truth instead of guesswork.
"""

import argparse
import json
import shutil
import subprocess
import tempfile
from pathlib import Path

import numpy as np
import pyarrow.parquet as pq

FPS = 30
ROBOT_TYPE = "synthetic_test"
STATE_DIM = 4
ACTION_DIM = 3
CAMERAS = {"observation.images.cam_a": (48, 64), "observation.images.cam_b": (48, 64)}
EPISODES = [
    ("pick up the cube", 45),
    ("pick up the cube", 40),
    ("place the cube in the bin", 50),
]


def features() -> dict:
    feats = {
        "observation.state": {
            "dtype": "float32",
            "shape": (STATE_DIM,),
            "names": [f"state_{i}" for i in range(STATE_DIM)],
        },
        "action": {
            "dtype": "float32",
            "shape": (ACTION_DIM,),
            "names": [f"action_{i}" for i in range(ACTION_DIM)],
        },
    }
    for key, (h, w) in CAMERAS.items():
        feats[key] = {
            "dtype": "video",
            "shape": (h, w, 3),
            "names": ["height", "width", "channels"],
        }
    return feats


def state_value(episode: int, frame: int, dim: int) -> np.float32:
    return np.float32(episode + frame / 100.0 + dim / 1000.0)


def action_value(episode: int, frame: int, dim: int) -> np.float32:
    return np.float32(-episode - frame / 100.0 - dim / 1000.0)


def image_value(episode: int, frame: int, h: int, w: int) -> np.ndarray:
    """Moving gradient, distinct per episode and frame."""
    ys = np.arange(h, dtype=np.uint16)[:, None]
    xs = np.arange(w, dtype=np.uint16)[None, :]
    img = np.empty((h, w, 3), dtype=np.uint8)
    img[..., 0] = ((ys * 3 + frame * 2) % 256).astype(np.uint8)
    img[..., 1] = ((xs * 3 + episode * 40) % 256).astype(np.uint8)
    img[..., 2] = ((ys + xs + frame) % 256).astype(np.uint8)
    return img


def generate(root: Path):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    ds = LeRobotDataset.create(
        repo_id="compliance/golden",
        fps=FPS,
        root=root,
        features=features(),
        robot_type=ROBOT_TYPE,
    )
    for episode, (task, length) in enumerate(EPISODES):
        for frame in range(length):
            row = {
                "observation.state": np.array(
                    [state_value(episode, frame, d) for d in range(STATE_DIM)],
                    dtype=np.float32,
                ),
                "action": np.array(
                    [action_value(episode, frame, d) for d in range(ACTION_DIM)],
                    dtype=np.float32,
                ),
                "task": task,
            }
            for key, (h, w) in CAMERAS.items():
                row[key] = image_value(episode, frame, h, w)
            ds.add_frame(row)
        ds.save_episode()
    if hasattr(ds, "finalize"):
        ds.finalize()
    return ds


def dump_parquet_schema(path: Path, out: Path, name: str):
    schema = pq.read_schema(path)
    doc = {
        "source": str(path.name),
        "fields": [
            {
                "name": f.name,
                "type": str(f.type),
                "nullable": f.nullable,
            }
            for f in schema
        ],
        "metadata": {
            (k.decode() if isinstance(k, bytes) else k): (
                v.decode() if isinstance(v, bytes) else v
            )
            for k, v in (schema.metadata or {}).items()
        },
    }
    (out / f"schema.{name}.json").write_text(json.dumps(doc, indent=2, sort_keys=True))


def dump_video_facts(video: Path, out: Path, name: str):
    probe = json.loads(
        subprocess.run(
            [
                "ffprobe",
                "-v",
                "error",
                "-select_streams",
                "v:0",
                "-show_streams",
                "-show_format",
                "-of",
                "json",
                str(video),
            ],
            check=True,
            capture_output=True,
        ).stdout
    )
    stream = probe["streams"][0]
    doc = {
        "codec_name": stream.get("codec_name"),
        "pix_fmt": stream.get("pix_fmt"),
        "width": stream.get("width"),
        "height": stream.get("height"),
        "r_frame_rate": stream.get("r_frame_rate"),
        "avg_frame_rate": stream.get("avg_frame_rate"),
        "time_base": stream.get("time_base"),
        "nb_frames": stream.get("nb_frames"),
    }
    (out / f"video.{name}.json").write_text(json.dumps(doc, indent=2, sort_keys=True))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument(
        "--keep-dataset",
        type=Path,
        default=None,
        help="also copy the full generated dataset to this directory",
    )
    args = parser.parse_args()

    import lerobot

    out = args.out
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)

    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp) / "golden"
        generate(root)

        shutil.copy(root / "meta" / "info.json", out / "info.json")
        shutil.copy(root / "meta" / "stats.json", out / "stats.json")
        dump_parquet_schema(
            next((root / "data").rglob("*.parquet")), out, "data"
        )
        dump_parquet_schema(
            next((root / "meta" / "episodes").rglob("*.parquet")), out, "episodes"
        )
        dump_parquet_schema(root / "meta" / "tasks.parquet", out, "tasks")
        for key in CAMERAS:
            camera = key.removeprefix("observation.images.")
            video = next((root / "videos" / key).rglob("*.mp4"))
            dump_video_facts(video, out, camera)

        if args.keep_dataset:
            if args.keep_dataset.exists():
                shutil.rmtree(args.keep_dataset)
            shutil.copytree(root, args.keep_dataset)

    manifest = {
        "lerobot_version": lerobot.__version__,
        "fps": FPS,
        "robot_type": ROBOT_TYPE,
        "state_dim": STATE_DIM,
        "action_dim": ACTION_DIM,
        "cameras": {k: list(v) for k, v in CAMERAS.items()},
        "episodes": [{"task": t, "length": n} for t, n in EPISODES],
    }
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True))
    print(f"golden fixtures written to {out}")


if __name__ == "__main__":
    main()
