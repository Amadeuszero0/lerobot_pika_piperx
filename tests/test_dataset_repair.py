import importlib.util
import json
import sys
from pathlib import Path

import pyarrow as pa
import pyarrow.parquet as pq


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "repair_lerobot_dataset.py"
SPEC = importlib.util.spec_from_file_location("repair_lerobot_dataset", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
repair = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = repair
SPEC.loader.exec_module(repair)


def _write_interrupted_dataset(
    root: Path,
    *,
    second_episode_complete: bool = True,
) -> None:
    (root / "meta").mkdir(parents=True)
    (root / "data" / "chunk-000").mkdir(parents=True)
    info = {
        "codebase_version": "v3.0",
        "robot_type": "dual_piper_follower",
        "fps": 30,
        "total_episodes": 2,
        "total_frames": 4,
        "total_tasks": 1,
        "splits": {"train": "0:2"},
        "features": {
            "action": {"dtype": "float32", "shape": [1], "names": ["joint"]},
            "timestamp": {"dtype": "float32", "shape": [1], "names": None},
            "frame_index": {"dtype": "int64", "shape": [1], "names": None},
            "episode_index": {"dtype": "int64", "shape": [1], "names": None},
            "index": {"dtype": "int64", "shape": [1], "names": None},
            "task_index": {"dtype": "int64", "shape": [1], "names": None},
        },
    }
    (root / "meta" / "info.json").write_text(json.dumps(info), encoding="utf-8")
    stats = {
        name: {
            "min": [0.0],
            "max": [1.0],
            "mean": [0.5],
            "std": [0.1],
            "count": [4],
        }
        for name in info["features"]
    }
    (root / "meta" / "stats.json").write_text(json.dumps(stats), encoding="utf-8")
    pq.write_table(
        pa.table({"task": ["test"], "task_index": [0]}),
        root / "meta" / "tasks.parquet",
    )
    second_frame_indices = [0, 1] if second_episode_complete else [0, 2]
    pq.write_table(
        pa.table(
            {
                "action": [[0.0], [0.1], [0.2], [0.3]],
                "timestamp": [0.0, 1 / 30, 0.0, 1 / 30],
                "frame_index": [0, 1, *second_frame_indices],
                "episode_index": [0, 0, 1, 1],
                "index": [0, 1, 2, 3],
                "task_index": [0, 0, 0, 0],
            }
        ),
        root / "data" / "chunk-000" / "file-000.parquet",
    )


def test_rebuilds_missing_episode_metadata(tmp_path: Path) -> None:
    root = tmp_path / "interrupted"
    _write_interrupted_dataset(root)

    plan = repair.build_repair_plan(root)

    assert plan.keep_episodes == 2
    assert plan.kept_frames == 4
    assert not (root / "meta" / "episodes").exists()

    backup = repair.apply_repair(plan, verify_loader=False)

    episodes = pq.read_table(root / "meta" / "episodes/chunk-000/file-000.parquet")
    assert episodes["episode_index"].to_pylist() == [0, 1]
    assert episodes["length"].to_pylist() == [2, 2]
    assert backup.is_dir()
    assert (root / "meta" / "repair_log.json").is_file()


def test_discards_only_invalid_trailing_episode(tmp_path: Path) -> None:
    root = tmp_path / "interrupted_tail"
    _write_interrupted_dataset(root, second_episode_complete=False)

    plan = repair.build_repair_plan(root)

    assert plan.valid_data_prefix == 1
    assert plan.keep_episodes == 1
    backup = repair.apply_repair(plan, verify_loader=False)

    info = json.loads((root / "meta" / "info.json").read_text(encoding="utf-8"))
    data = pq.read_table(root / "data/chunk-000/file-000.parquet")
    episodes = pq.read_table(root / "meta/episodes/chunk-000/file-000.parquet")
    assert info["total_episodes"] == 1
    assert info["total_frames"] == 2
    assert data["episode_index"].to_pylist() == [0, 0]
    assert episodes["episode_index"].to_pylist() == [0]
    assert (backup / "data/chunk-000/file-000.parquet").is_file()
