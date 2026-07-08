"""Compliance gate: loads a dataset written by the lerobot_dataset crate with
the Python lerobot loader and asserts frame-level correctness.

Usage: pixi run check <dataset_root>

The dataset must be the synthetic fixture from `cargo run --example
generate_fixture` (closed-form values are re-derived here). Every frame of
every episode is iterated, which exercises parquet reading, task resolution,
video decode, and the loader's PTS tolerance check.
"""

import sys
from pathlib import Path

import numpy as np
import torch

FPS = 30
STATE_DIM = 4
ACTION_DIM = 3
WIDTH, HEIGHT = 64, 48
CAMERAS = ["observation.images.cam_a", "observation.images.cam_b"]
EPISODES = [
    ("pick up the cube", 45),
    ("pick up the cube", 40),
    ("place the cube in the bin", 50),
]


def state_value(episode: int, frame: int, dim: int) -> np.float32:
    return np.float32(
        np.float32(episode)
        + np.float32(frame) / np.float32(100.0)
        + np.float32(dim) / np.float32(1000.0)
    )


def action_value(episode: int, frame: int, dim: int) -> np.float32:
    return -state_value(episode, frame, dim)


def expected_image(episode: int, frame: int) -> np.ndarray:
    """Mirror of examples/generate_fixture.rs: a white 4px bar marching 4px
    per frame over an episode-coded background."""
    background = 30 + episode * 40
    xs = np.arange(WIDTH, dtype=np.int64)[None, :]
    in_bar = (xs - frame * 4) % WIDTH < 4
    column = np.where(in_bar, 255.0, float(background))
    img = np.broadcast_to(column[..., None], (HEIGHT, WIDTH, 3)).astype(np.float32)
    return img / 255.0


def to_hwc(chw: torch.Tensor) -> np.ndarray:
    return chw.permute(1, 2, 0).numpy()


def check_frames(ds) -> None:
    row = 0
    for episode, (task, length) in enumerate(EPISODES):
        for frame in range(length):
            item = ds[row]
            state = item["observation.state"].numpy()
            action = item["action"].numpy()
            expected_state = np.array(
                [state_value(episode, frame, d) for d in range(STATE_DIM)], dtype=np.float32
            )
            expected_action = np.array(
                [action_value(episode, frame, d) for d in range(ACTION_DIM)], dtype=np.float32
            )
            np.testing.assert_array_equal(state, expected_state, err_msg=f"state row {row}")
            np.testing.assert_array_equal(action, expected_action, err_msg=f"action row {row}")
            assert item["episode_index"].item() == episode, row
            assert item["frame_index"].item() == frame, row
            assert item["index"].item() == row, row
            assert abs(item["timestamp"].item() - frame / FPS) < 1e-6, row
            assert item["task"] == task, (row, item["task"])

            for key in CAMERAS:
                decoded = to_hwc(item[key])
                assert decoded.shape == (HEIGHT, WIDTH, 3), (key, decoded.shape)
                mae_here = np.abs(decoded - expected_image(episode, frame)).mean()
                # Alignment check: the decoded frame must match its own index
                # clearly better than any nearby frame.
                for off in (-2, -1, 1, 2):
                    neighbor = frame + off
                    if 0 <= neighbor < length:
                        mae_neighbor = np.abs(
                            decoded - expected_image(episode, neighbor)
                        ).mean()
                        assert mae_here < mae_neighbor, (
                            f"{key} row {row}: frame misaligned "
                            f"(mae {mae_here:.4f} vs neighbor {mae_neighbor:.4f})"
                        )
                assert mae_here < 0.05, f"{key} row {row}: mae {mae_here:.4f} too high"
            row += 1


def check_stats(ds) -> None:
    from lerobot.datasets.compute_stats import RunningQuantileStats

    stats = ds.meta.stats
    total = sum(n for _, n in EPISODES)

    all_states = np.array(
        [
            [state_value(e, f, d) for d in range(STATE_DIM)]
            for e, (_, n) in enumerate(EPISODES)
            for f in range(n)
        ],
        dtype=np.float32,
    )
    np.testing.assert_allclose(
        stats["observation.state"]["mean"],
        all_states.mean(axis=0),
        rtol=0,
        atol=1e-5,
        err_msg="state mean",
    )
    np.testing.assert_allclose(
        stats["observation.state"]["min"], all_states.min(axis=0), atol=1e-6
    )
    np.testing.assert_allclose(
        stats["observation.state"]["max"], all_states.max(axis=0), atol=1e-6
    )
    assert stats["observation.state"]["count"][0] == total

    # Aggregated std must match lerobot's own parallel aggregation applied to
    # per-episode population stats.
    per_episode = []
    start = 0
    for _, n in EPISODES:
        chunk = all_states[start : start + n]
        per_episode.append((chunk.mean(axis=0), chunk.std(axis=0), n))
        start += n
    agg_mean = sum(m * n for m, _, n in per_episode) / total
    agg_var = sum((s**2 + (m - agg_mean) ** 2) * n for m, s, n in per_episode) / total
    np.testing.assert_allclose(
        stats["observation.state"]["std"], np.sqrt(agg_var), rtol=0, atol=1e-5
    )

    for key in CAMERAS:
        for stat in ["min", "max", "mean", "std", "q01", "q10", "q50", "q90", "q99"]:
            value = np.asarray(stats[key][stat])
            assert value.shape == (3, 1, 1), (key, stat, value.shape)
            assert (value >= -1e-9).all() and (value <= 1.0 + 1e-9).all(), (key, stat)
        assert stats[key]["count"][0] > 0

    for key in ["timestamp", "frame_index", "episode_index", "index", "task_index"]:
        assert set(stats[key]) >= {"min", "max", "mean", "std", "count"}, key

    # Quantile parity: replicate lerobot's own histogram estimator over
    # episode 0 and compare with the recorded per-episode stats. The loader
    # drops stats/ columns, so read the episodes parquet directly.
    import pandas as pd

    episodes_df = pd.read_parquet(ds.root / "meta" / "episodes" / "chunk-000" / "file-000.parquet")
    rqs = RunningQuantileStats()
    rqs.update(all_states[: EPISODES[0][1]].astype(np.float64))
    expected = rqs.get_statistics()
    for stat in ["q01", "q10", "q50", "q90", "q99", "mean", "std", "min", "max"]:
        recorded = np.asarray(episodes_df.iloc[0][f"stats/observation.state/{stat}"])
        np.testing.assert_allclose(
            recorded, expected[stat], rtol=0, atol=1e-5,
            err_msg=f"episode-0 {stat} parity with lerobot's estimator",
        )


def main() -> None:
    root = Path(sys.argv[1]).resolve()
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    ds = LeRobotDataset("compliance/native", root=root)
    assert ds.meta.total_episodes == len(EPISODES), ds.meta.total_episodes
    assert ds.meta.total_frames == sum(n for _, n in EPISODES), ds.meta.total_frames
    assert ds.meta.fps == FPS
    assert len(ds) == ds.meta.total_frames, (len(ds), ds.meta.total_frames)
    assert ds.meta.total_tasks == 2

    check_frames(ds)
    check_stats(ds)
    print(f"compliance ok: {ds.meta.total_episodes} episodes, {len(ds)} frames, "
          f"{len(CAMERAS)} cameras decoded and aligned")


if __name__ == "__main__":
    main()
