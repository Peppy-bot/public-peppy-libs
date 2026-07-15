"""Depth compliance gate: loads a Rust-written dataset that has a depth
camera and asserts the Python lerobot loader dequantizes it back to the
millimetre values the fixture wrote.

Usage: pixi run check-depth <dataset_root>

The dataset must be from `cargo run --example generate_depth_fixture`.
"""

import sys
from pathlib import Path

import numpy as np

FPS = 10
WIDTH, HEIGHT = 32, 24
EPISODES = [("reach", 12), ("grasp", 15)]


def depth_mm(episode: int, frame: int) -> int:
    return 600 + episode * 400 + frame * 120


def main() -> None:
    root = Path(sys.argv[1]).resolve()
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    ds = LeRobotDataset("compliance/depth", root=root)
    total = sum(n for _, n in EPISODES)
    assert len(ds) == total, (len(ds), total)

    # info.json must mark the depth feature and carry the quantization params.
    depth_info = ds.meta.info.features["observation.images.depth"]["info"]
    assert depth_info["is_depth_map"] is True, depth_info
    assert depth_info["video.codec"] == "hevc", depth_info
    assert depth_info["video.pix_fmt"] == "gray12le", depth_info
    assert depth_info["depth_unit"] == "mm", depth_info
    for k, v in [("video.depth_min", 0.01), ("video.depth_max", 10.0),
                 ("video.shift", 3.5), ("video.use_log", True)]:
        assert depth_info[k] == v, (k, depth_info.get(k))

    # color feature stays RGB.
    color_info = ds.meta.info.features["observation.images.color"]["info"]
    assert color_info["is_depth_map"] is False, color_info

    row = 0
    max_err = 0.0
    for episode, (_task, length) in enumerate(EPISODES):
        for frame in range(length):
            item = ds[row]
            depth = item["observation.images.depth"].float().numpy()  # (1,H,W) mm
            assert depth.shape == (1, HEIGHT, WIDTH), depth.shape
            expected = depth_mm(episode, frame)
            err = float(np.abs(depth - expected).max())
            max_err = max(max_err, err)
            # 12-bit log quantization over 0.01..10 m: the step near ~1 m is
            # well under 2 mm; allow a small tolerance.
            assert err < 3.0, f"row {row}: depth err {err:.2f} mm at expected {expected}"
            row += 1

    # depth stats present and shaped (1,1,1) in mm.
    stats = ds.meta.stats["observation.images.depth"]
    for key in ["min", "max", "mean", "std"]:
        assert np.asarray(stats[key]).shape == (1, 1, 1), (key, np.asarray(stats[key]).shape)
    assert stats["min"][0][0][0] >= 500, stats["min"]

    print(f"depth compliance ok: {total} frames, max dequant error {max_err:.2f} mm")


if __name__ == "__main__":
    main()
