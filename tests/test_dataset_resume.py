import importlib.util
import json
import sys
from pathlib import Path

import pytest


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "inspect_lerobot_resume.py"
SPEC = importlib.util.spec_from_file_location("inspect_lerobot_resume", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
resume_module = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = resume_module
SPEC.loader.exec_module(resume_module)


def _write_dataset(root: Path, *, episodes: int, frames: int = 0) -> None:
    (root / "meta").mkdir(parents=True)
    info = {
        "codebase_version": "v3.0",
        "robot_type": "dual_piper_follower",
        "fps": 30,
        "features": {"observation.state": {"dtype": "float32", "shape": [26]}},
        "total_episodes": episodes,
        "total_frames": frames,
    }
    (root / "meta" / "info.json").write_text(json.dumps(info), encoding="utf-8")
    if episodes:
        (root / "data").mkdir()
        (root / "meta" / "episodes").mkdir()
        (root / "meta" / "tasks.parquet").touch()


def test_resume_plan_records_only_missing_episodes(tmp_path: Path) -> None:
    root = tmp_path / "dual_pika_dataset_20260814_120000"
    _write_dataset(root, episodes=12, frames=3600)

    plan = resume_module.build_resume_plan(root, target_episodes=50)

    assert plan.root == root.resolve()
    assert plan.repo_id == f"local/{root.name}"
    assert plan.saved_episodes == 12
    assert plan.episodes_to_record == 38


def test_resume_plan_rejects_completed_target(tmp_path: Path) -> None:
    root = tmp_path / "complete_dataset"
    _write_dataset(root, episodes=50, frames=15000)

    with pytest.raises(ValueError, match="meets or exceeds"):
        resume_module.build_resume_plan(root, target_episodes=50)


def test_resume_plan_rejects_incomplete_saved_dataset(tmp_path: Path) -> None:
    root = tmp_path / "incomplete_dataset"
    _write_dataset(root, episodes=3, frames=900)
    (root / "meta" / "tasks.parquet").unlink()

    with pytest.raises(ValueError, match="saved dataset is incomplete"):
        resume_module.build_resume_plan(root, target_episodes=50)


def test_formal_launcher_uses_explicit_resume_mode() -> None:
    launcher = (
        Path(__file__).resolve().parents[1]
        / "scripts"
        / "start_dual_pika_piper_recording.sh"
    ).read_text(encoding="utf-8")

    assert "--resume PATH" in launcher
    assert '--num-episodes "${episodes_this_run}"' in launcher
    assert 'record_args=(--resume "${record_args[@]}")' in launcher
    assert "Type ${required_confirmation}" in launcher
