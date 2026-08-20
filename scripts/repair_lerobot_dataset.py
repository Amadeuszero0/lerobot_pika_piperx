#!/usr/bin/env python3
"""Repair an interrupted local LeRobot v3 dataset so recording can resume.

The tool is deliberately conservative:

* the default mode is read-only and prints a repair plan;
* only a contiguous, internally valid prefix of episodes is kept;
* an episode is considered recoverable only when its data and every configured
  video stream contain all of its frames;
* files changed by ``--apply`` are copied to a timestamped backup first;
* trailing incomplete data/video/image artifacts are removed from the active
  dataset only after the backup has been created.

It targets the LeRobot v3 layout used by lerobot==0.4.3.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime
from fractions import Fraction
from pathlib import Path
from typing import Any, Callable

import numpy as np
import pyarrow as pa
import pyarrow.compute as pc
import pyarrow.parquet as pq


CHUNK_FILE_RE = re.compile(r"chunk-(\d+)/file-(\d+)\.(?:parquet|mp4)$")
TEMP_DIR_RE = re.compile(r"^tmp[0-9A-Za-z_-]+$")
DEFAULT_STATS = ("min", "max", "mean", "std", "count")


@dataclass(frozen=True)
class DataFile:
    path: Path
    chunk_index: int
    file_index: int
    table: pa.Table


@dataclass(frozen=True)
class DataEpisode:
    episode_index: int
    length: int
    data_file: DataFile
    row_indices: tuple[int, ...]
    dataset_from_index: int
    dataset_to_index: int
    task_indices: tuple[int, ...]


@dataclass(frozen=True)
class VideoFile:
    path: Path
    chunk_index: int
    file_index: int
    frames: int
    fps: float
    duration_s: float
    codec: str


@dataclass(frozen=True)
class VideoLocation:
    video_file: VideoFile
    from_frame: int
    to_frame: int


@dataclass(frozen=True)
class RepairPlan:
    root: Path
    info: dict[str, Any]
    stats: dict[str, Any]
    task_names: dict[int, str]
    data_files: tuple[DataFile, ...]
    data_episodes: tuple[DataEpisode, ...]
    valid_data_prefix: int
    video_files: dict[str, tuple[VideoFile, ...]]
    video_locations: dict[str, dict[int, VideoLocation]]
    video_complete_prefix: dict[str, int]
    keep_episodes: int

    @property
    def kept_frames(self) -> int:
        return sum(episode.length for episode in self.data_episodes[: self.keep_episodes])

    @property
    def reported_episodes(self) -> int:
        return int(self.info.get("total_episodes", 0))

    @property
    def reported_frames(self) -> int:
        return int(self.info.get("total_frames", 0))


def _read_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise ValueError(f"missing required file: {path}")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid JSON in {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def _chunk_file_indices(path: Path, root: Path) -> tuple[int, int]:
    relative = path.relative_to(root).as_posix()
    match = CHUNK_FILE_RE.search(relative)
    if match is None:
        raise ValueError(f"unexpected LeRobot chunk filename: {relative}")
    return int(match.group(1)), int(match.group(2))


def _load_tasks(root: Path) -> dict[int, str]:
    path = root / "meta" / "tasks.parquet"
    if not path.is_file():
        raise ValueError(f"missing required task metadata: {path}")
    table = pq.read_table(path)
    if "task_index" not in table.column_names:
        raise ValueError(f"{path} has no task_index column")

    name_column = "task" if "task" in table.column_names else None
    if name_column is None:
        for candidate in table.column_names:
            if candidate == "task_index":
                continue
            values = table[candidate].to_pylist()
            if all(isinstance(value, str) for value in values):
                name_column = candidate
                break
    if name_column is None:
        raise ValueError(f"unable to find task text column in {path}")

    indices = [int(value) for value in table["task_index"].to_pylist()]
    names = [str(value) for value in table[name_column].to_pylist()]
    if len(set(indices)) != len(indices):
        raise ValueError("tasks.parquet contains duplicate task_index values")
    return dict(zip(indices, names, strict=True))


def _load_data_files(root: Path) -> tuple[DataFile, ...]:
    paths = sorted((root / "data").glob("chunk-*/*.parquet"))
    if not paths:
        raise ValueError(f"no data parquet files found below {root / 'data'}")

    result: list[DataFile] = []
    required = {"episode_index", "frame_index", "index", "timestamp", "task_index"}
    for path in paths:
        try:
            table = pq.read_table(path)
        except Exception as exc:
            raise ValueError(
                f"cannot read {path}; its Parquet footer may be incomplete: {exc}"
            ) from exc
        missing = sorted(required - set(table.column_names))
        if missing:
            raise ValueError(f"{path} is missing columns: {', '.join(missing)}")
        chunk_index, file_index = _chunk_file_indices(path, root / "data")
        result.append(DataFile(path, chunk_index, file_index, table))
    return tuple(result)


def _scan_data_episodes(
    data_files: tuple[DataFile, ...], task_names: dict[int, str]
) -> tuple[tuple[DataEpisode, ...], int]:
    locations: dict[int, list[tuple[DataFile, int]]] = {}
    for data_file in data_files:
        for row, value in enumerate(data_file.table["episode_index"].to_pylist()):
            locations.setdefault(int(value), []).append((data_file, row))

    if not locations:
        raise ValueError("data parquet contains no frames")
    maximum = max(locations)
    episodes: list[DataEpisode] = []
    valid_prefix = 0

    for episode_index in range(maximum + 1):
        rows = locations.get(episode_index)
        if not rows:
            break
        files = {row[0].path for row in rows}
        if len(files) != 1:
            break
        data_file = rows[0][0]
        row_indices = tuple(row for _, row in rows)
        take = pa.array(row_indices, type=pa.int64())
        episode_table = data_file.table.take(take)
        order = np.argsort(
            np.asarray(episode_table["frame_index"].to_pylist(), dtype=np.int64),
            kind="stable",
        )
        episode_table = episode_table.take(pa.array(order, type=pa.int64()))
        sorted_rows = tuple(row_indices[int(position)] for position in order)

        frame_indices = np.asarray(episode_table["frame_index"].to_pylist(), dtype=np.int64)
        global_indices = np.asarray(episode_table["index"].to_pylist(), dtype=np.int64)
        timestamps = np.asarray(episode_table["timestamp"].to_pylist(), dtype=np.float64)
        tasks = tuple(
            dict.fromkeys(
                int(value) for value in episode_table["task_index"].to_pylist()
            )
        )
        length = len(frame_indices)
        expected_start = episodes[-1].dataset_to_index if episodes else 0
        valid = (
            length > 0
            and np.array_equal(frame_indices, np.arange(length, dtype=np.int64))
            and np.array_equal(
                global_indices,
                np.arange(expected_start, expected_start + length, dtype=np.int64),
            )
            and np.isfinite(timestamps).all()
            and (length < 2 or bool(np.all(np.diff(timestamps) > 0)))
            and all(task in task_names for task in tasks)
        )

        episodes.append(
            DataEpisode(
                episode_index=episode_index,
                length=length,
                data_file=data_file,
                row_indices=sorted_rows,
                dataset_from_index=expected_start,
                dataset_to_index=expected_start + length,
                task_indices=tasks,
            )
        )
        if not valid:
            break
        valid_prefix += 1

    return tuple(episodes), valid_prefix


def _parse_fraction(value: str) -> float:
    if value in {"", "0/0", "N/A"}:
        return 0.0
    try:
        return float(Fraction(value))
    except (ValueError, ZeroDivisionError):
        return float(value)


def probe_video(path: Path, ffprobe: str = "ffprobe") -> VideoFile:
    command = [
        ffprobe,
        "-v",
        "error",
        "-count_frames",
        "-select_streams",
        "v:0",
        "-show_entries",
        "stream=nb_read_frames,nb_frames,avg_frame_rate,duration,codec_name:format=duration",
        "-of",
        "json",
        str(path),
    ]
    try:
        result = subprocess.run(command, check=True, capture_output=True, text=True)
    except FileNotFoundError as exc:
        raise ValueError("ffprobe is required to validate video files") from exc
    except subprocess.CalledProcessError as exc:
        raise ValueError(f"ffprobe failed for {path}: {exc.stderr.strip()}") from exc

    payload = json.loads(result.stdout)
    streams = payload.get("streams", [])
    if len(streams) != 1:
        raise ValueError(f"expected one video stream in {path}")
    stream = streams[0]
    frame_text = stream.get("nb_read_frames") or stream.get("nb_frames")
    if frame_text in {None, "N/A"}:
        raise ValueError(f"ffprobe could not count frames in {path}")
    frames = int(frame_text)
    fps = _parse_fraction(str(stream.get("avg_frame_rate", "0/0")))
    duration = float(stream.get("duration") or payload.get("format", {}).get("duration") or 0.0)
    codec = str(stream.get("codec_name") or "")
    if frames <= 0 or fps <= 0 or not math.isfinite(duration):
        raise ValueError(f"invalid video properties for {path}")
    return VideoFile(path, 0, 0, frames, fps, duration, codec)


def _load_video_files(
    root: Path,
    video_keys: list[str],
    dataset_fps: float,
    probe: Callable[[Path], VideoFile],
) -> dict[str, tuple[VideoFile, ...]]:
    result: dict[str, tuple[VideoFile, ...]] = {}
    for key in video_keys:
        paths = sorted((root / "videos" / key).glob("chunk-*/*.mp4"))
        files: list[VideoFile] = []
        for path in paths:
            value = probe(path)
            chunk_index, file_index = _chunk_file_indices(path, root / "videos" / key)
            if not math.isclose(value.fps, dataset_fps, abs_tol=0.05):
                raise ValueError(
                    f"{path} is {value.fps:.4f} fps but dataset metadata says {dataset_fps:.4f}"
                )
            files.append(
                VideoFile(
                    path=path,
                    chunk_index=chunk_index,
                    file_index=file_index,
                    frames=value.frames,
                    fps=value.fps,
                    duration_s=value.duration_s,
                    codec=value.codec,
                )
            )
        result[key] = tuple(files)
    return result


def _assign_video_episodes(
    episodes: tuple[DataEpisode, ...], video_files: tuple[VideoFile, ...]
) -> tuple[dict[int, VideoLocation], int]:
    locations: dict[int, VideoLocation] = {}
    cursor = 0
    partial_seen = False
    for video_file in video_files:
        if partial_seen:
            raise ValueError(
                f"video files continue after a partial episode in {video_file.path.parent}"
            )
        used = 0
        while cursor < len(episodes) and video_file.frames - used >= episodes[cursor].length:
            length = episodes[cursor].length
            locations[cursor] = VideoLocation(video_file, used, used + length)
            used += length
            cursor += 1
        if used != video_file.frames:
            partial_seen = True
    return locations, cursor


def build_repair_plan(
    dataset_root: str | Path,
    *,
    video_probe: Callable[[Path], VideoFile] = probe_video,
) -> RepairPlan:
    root = Path(dataset_root).expanduser().resolve(strict=True)
    if not root.is_dir():
        raise ValueError(f"dataset root is not a directory: {root}")
    info = _read_json(root / "meta" / "info.json")
    stats = _read_json(root / "meta" / "stats.json")
    task_names = _load_tasks(root)
    data_files = _load_data_files(root)
    data_episodes, valid_data_prefix = _scan_data_episodes(data_files, task_names)
    if valid_data_prefix == 0:
        raise ValueError("no complete episode can be recovered safely")

    fps = float(info.get("fps", 0))
    if not math.isfinite(fps) or fps <= 0:
        raise ValueError("info.json fps must be finite and positive")
    features = info.get("features")
    if not isinstance(features, dict) or not features:
        raise ValueError("info.json features must be a non-empty mapping")
    missing_stats = sorted(
        key
        for key, feature in features.items()
        if feature.get("dtype") not in {"string", "language"} and key not in stats
    )
    if missing_stats:
        raise ValueError(
            "stats.json is missing feature statistics required to rebuild a resume-safe "
            f"episode schema: {', '.join(missing_stats)}"
        )
    video_keys = [key for key, feature in features.items() if feature.get("dtype") == "video"]
    video_files = _load_video_files(root, video_keys, fps, video_probe)
    video_locations: dict[str, dict[int, VideoLocation]] = {}
    video_complete_prefix: dict[str, int] = {}
    for key in video_keys:
        locations, complete_prefix = _assign_video_episodes(data_episodes, video_files[key])
        video_locations[key] = locations
        video_complete_prefix[key] = complete_prefix

    reported_episodes = int(info.get("total_episodes", 0))
    keep_episodes = min(
        [reported_episodes, valid_data_prefix, *video_complete_prefix.values()]
        if video_keys
        else [reported_episodes, valid_data_prefix]
    )
    if keep_episodes == 0:
        raise ValueError("no episode has complete data and complete video for every camera")

    return RepairPlan(
        root=root,
        info=info,
        stats=stats,
        task_names=task_names,
        data_files=data_files,
        data_episodes=data_episodes,
        valid_data_prefix=valid_data_prefix,
        video_files=video_files,
        video_locations=video_locations,
        video_complete_prefix=video_complete_prefix,
        keep_episodes=keep_episodes,
    )


def _numeric_stats(values: Any, stat_names: tuple[str, ...]) -> dict[str, list[Any]]:
    array = np.asarray(values)
    if array.dtype.kind not in "biuf":
        raise ValueError(f"cannot compute statistics for dtype {array.dtype}")
    array = array.astype(np.float64, copy=False)
    reduce_axis = 0
    result: dict[str, list[Any]] = {}
    for name in stat_names:
        if name == "min":
            value = np.min(array, axis=reduce_axis)
        elif name == "max":
            value = np.max(array, axis=reduce_axis)
        elif name == "mean":
            value = np.mean(array, axis=reduce_axis)
        elif name == "std":
            value = np.std(array, axis=reduce_axis)
        elif name == "count":
            value = np.asarray([len(array)], dtype=np.int64)
        elif name.startswith("q") and name[1:].isdigit():
            value = np.quantile(array, int(name[1:]) / 100.0, axis=reduce_axis)
        else:
            continue
        result[name] = np.atleast_1d(value).tolist()
    return result


def _episode_table(episode: DataEpisode) -> pa.Table:
    return episode.data_file.table.take(pa.array(episode.row_indices, type=pa.int64()))


def _stat_names(stats: dict[str, Any], feature: str) -> tuple[str, ...]:
    observed = stats.get(feature)
    if isinstance(observed, dict) and observed:
        return tuple(str(name) for name in observed)
    return DEFAULT_STATS


def _build_episode_metadata(plan: RepairPlan) -> pa.Table:
    rows: list[dict[str, Any]] = []
    features = plan.info["features"]
    fps = float(plan.info["fps"])
    for episode in plan.data_episodes[: plan.keep_episodes]:
        table = _episode_table(episode)
        row: dict[str, Any] = {
            "episode_index": episode.episode_index,
            "tasks": [plan.task_names[index] for index in episode.task_indices],
            "length": episode.length,
            "data/chunk_index": episode.data_file.chunk_index,
            "data/file_index": episode.data_file.file_index,
            "dataset_from_index": episode.dataset_from_index,
            "dataset_to_index": episode.dataset_to_index,
            "meta/episodes/chunk_index": 0,
            "meta/episodes/file_index": 0,
        }
        for key, feature in features.items():
            dtype = feature.get("dtype")
            if dtype in {"video", "image", "string", "language"}:
                feature_stats = plan.stats.get(key, {})
                if isinstance(feature_stats, dict):
                    for stat_name, value in feature_stats.items():
                        row[f"stats/{key}/{stat_name}"] = value
                continue
            if key not in table.column_names:
                continue
            values = table[key].to_pylist()
            for stat_name, value in _numeric_stats(
                values, _stat_names(plan.stats, key)
            ).items():
                row[f"stats/{key}/{stat_name}"] = value

        for key, locations in plan.video_locations.items():
            location = locations[episode.episode_index]
            row[f"videos/{key}/chunk_index"] = location.video_file.chunk_index
            row[f"videos/{key}/file_index"] = location.video_file.file_index
            row[f"videos/{key}/from_timestamp"] = location.from_frame / fps
            row[f"videos/{key}/to_timestamp"] = location.to_frame / fps
        rows.append(row)
    return pa.Table.from_pylist(rows)


def _build_global_stats(plan: RepairPlan) -> dict[str, Any]:
    kept_tables = [_episode_table(ep) for ep in plan.data_episodes[: plan.keep_episodes]]
    combined = pa.concat_tables(kept_tables)
    result = json.loads(json.dumps(plan.stats))
    for key, feature in plan.info["features"].items():
        if feature.get("dtype") in {"video", "image", "string", "language"}:
            continue
        if key not in combined.column_names:
            continue
        result[key] = _numeric_stats(
            combined[key].to_pylist(), _stat_names(plan.stats, key)
        )
    return result


def _write_json_atomic(path: Path, value: dict[str, Any]) -> None:
    temporary = path.with_name(f".{path.name}.repair.tmp")
    temporary.write_text(
        json.dumps(value, indent=4, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    temporary.replace(path)


def _trim_video(
    source: Path,
    output: Path,
    frames: int,
    expected_fps: float,
    codec: str,
    ffmpeg: str,
    ffprobe: str,
) -> None:
    if frames <= 0:
        raise ValueError("trim target must contain at least one frame")
    copy_command = [
        ffmpeg,
        "-v",
        "error",
        "-y",
        "-i",
        str(source),
        "-map",
        "0:v:0",
        "-frames:v",
        str(frames),
        "-an",
        "-c:v",
        "copy",
        "-movflags",
        "+faststart",
        str(output),
    ]
    try:
        subprocess.run(copy_command, check=True, capture_output=True, text=True)
        if probe_video(output, ffprobe).frames == frames:
            return
    except FileNotFoundError as exc:
        raise ValueError("ffmpeg is required to trim an incomplete trailing video") from exc
    except (subprocess.CalledProcessError, ValueError):
        pass
    output.unlink(missing_ok=True)

    encoder = {"av1": "libsvtav1", "h264": "libx264", "hevc": "libx265"}.get(codec)
    if encoder is None:
        raise ValueError(
            f"cannot safely trim codec {codec!r} in {source}; restore from the backup if needed"
        )
    encode_command = [
        ffmpeg,
        "-v",
        "error",
        "-y",
        "-i",
        str(source),
        "-map",
        "0:v:0",
        "-frames:v",
        str(frames),
        "-an",
        "-c:v",
        encoder,
        "-r",
        f"{expected_fps:g}",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(output),
    ]
    try:
        subprocess.run(encode_command, check=True, capture_output=True, text=True)
    except FileNotFoundError as exc:
        raise ValueError("ffmpeg is required to trim an incomplete trailing video") from exc
    except subprocess.CalledProcessError as exc:
        raise ValueError(f"ffmpeg failed to trim {source}: {exc.stderr.strip()}") from exc
    checked = probe_video(output, ffprobe)
    if checked.frames != frames:
        raise ValueError(
            f"trimmed video {output} has {checked.frames} frames; expected {frames}"
        )


def _backup_path(backup: Path, source: Path, root: Path) -> Path:
    return backup / source.relative_to(root)


def _copy_to_backup(backup: Path, source: Path, root: Path) -> None:
    destination = _backup_path(backup, source, root)
    destination.parent.mkdir(parents=True, exist_ok=True)
    if source.is_dir():
        shutil.copytree(source, destination, dirs_exist_ok=True)
    elif source.exists():
        shutil.copy2(source, destination)


def apply_repair(
    plan: RepairPlan,
    *,
    ffmpeg: str = "ffmpeg",
    ffprobe: str = "ffprobe",
    verify_loader: bool = True,
) -> Path:
    root = plan.root
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = root / f".repair_backup_{timestamp}"
    if backup.exists():
        raise ValueError(f"backup path already exists: {backup}")

    kept_ids = set(range(plan.keep_episodes))
    data_replacements: dict[Path, Path | None] = {}
    for data_file in plan.data_files:
        mask = pc.is_in(
            data_file.table["episode_index"],
            value_set=pa.array(sorted(kept_ids), type=pa.int64()),
        )
        kept_table = data_file.table.filter(mask)
        if kept_table.num_rows == data_file.table.num_rows:
            continue
        if kept_table.num_rows == 0:
            data_replacements[data_file.path] = None
            continue
        temporary = data_file.path.with_name(f".{data_file.path.name}.repair.tmp")
        pq.write_table(kept_table, temporary, compression="snappy", use_dictionary=True)
        pq.read_metadata(temporary)
        data_replacements[data_file.path] = temporary

    video_replacements: dict[Path, Path | None] = {}
    for key, files in plan.video_files.items():
        target_by_file: dict[Path, int] = {}
        for episode_index in range(plan.keep_episodes):
            location = plan.video_locations[key][episode_index]
            target_by_file[location.video_file.path] = (
                target_by_file.get(location.video_file.path, 0)
                + location.to_frame
                - location.from_frame
            )
        for video_file in files:
            target = target_by_file.get(video_file.path, 0)
            if target == video_file.frames:
                continue
            if target == 0:
                video_replacements[video_file.path] = None
                continue
            temporary = video_file.path.with_name(
                f".{video_file.path.stem}.repair.tmp.mp4"
            )
            _trim_video(
                video_file.path,
                temporary,
                target,
                float(plan.info["fps"]),
                video_file.codec,
                ffmpeg,
                ffprobe,
            )
            video_replacements[video_file.path] = temporary

    episodes_table = _build_episode_metadata(plan)
    episodes_path = root / "meta" / "episodes" / "chunk-000" / "file-000.parquet"
    episodes_temporary = root / "meta" / ".episodes.repair.tmp.parquet"
    pq.write_table(episodes_table, episodes_temporary, compression="snappy", use_dictionary=True)
    pq.read_metadata(episodes_temporary)

    new_info = json.loads(json.dumps(plan.info))
    new_info["total_episodes"] = plan.keep_episodes
    new_info["total_frames"] = plan.kept_frames
    new_info["total_tasks"] = len(plan.task_names)
    new_info["splits"] = {"train": f"0:{plan.keep_episodes}"}
    new_stats = _build_global_stats(plan)

    backup.mkdir(parents=False)
    for path in (root / "meta" / "info.json", root / "meta" / "stats.json"):
        _copy_to_backup(backup, path, root)
    _copy_to_backup(backup, root / "meta" / "episodes", root)
    for path in [*data_replacements, *video_replacements]:
        _copy_to_backup(backup, path, root)

    for original, replacement in data_replacements.items():
        if replacement is None:
            original.unlink(missing_ok=True)
        else:
            replacement.replace(original)
    for original, replacement in video_replacements.items():
        if replacement is None:
            original.unlink(missing_ok=True)
        else:
            replacement.replace(original)

    episodes_dir = root / "meta" / "episodes"
    if episodes_dir.exists():
        shutil.rmtree(episodes_dir)
    episodes_path.parent.mkdir(parents=True, exist_ok=True)
    episodes_temporary.replace(episodes_path)
    _write_json_atomic(root / "meta" / "stats.json", new_stats)
    _write_json_atomic(root / "meta" / "info.json", new_info)

    orphan_root = backup / "orphaned_staging"
    for path in root.iterdir():
        if path == backup:
            continue
        if path.is_dir() and TEMP_DIR_RE.fullmatch(path.name):
            orphan_root.mkdir(parents=True, exist_ok=True)
            shutil.move(str(path), str(orphan_root / path.name))
    images_root = root / "images"
    if images_root.is_dir():
        for episode_dir in images_root.glob("*/episode-*"):
            try:
                episode_index = int(episode_dir.name.removeprefix("episode-"))
            except ValueError:
                continue
            if episode_index >= plan.keep_episodes:
                destination = orphan_root / episode_dir.relative_to(root)
                destination.parent.mkdir(parents=True, exist_ok=True)
                shutil.move(str(episode_dir), str(destination))

    repair_log = {
        "repaired_at": datetime.now().astimezone().isoformat(),
        "backup": str(backup),
        "reported_before": {
            "episodes": plan.reported_episodes,
            "frames": plan.reported_frames,
        },
        "repaired": {"episodes": plan.keep_episodes, "frames": plan.kept_frames},
        "video_complete_prefix": plan.video_complete_prefix,
    }
    _write_json_atomic(root / "meta" / "repair_log.json", repair_log)

    if verify_loader:
        try:
            from lerobot.datasets.lerobot_dataset import LeRobotDataset

            dataset = LeRobotDataset(f"local/{root.name}", root=root)
            if int(dataset.num_episodes) != plan.keep_episodes:
                raise ValueError(
                    f"loader reports {dataset.num_episodes} episodes; expected {plan.keep_episodes}"
                )
            if len(dataset.hf_dataset) != plan.kept_frames:
                raise ValueError(
                    f"loader reports {len(dataset.hf_dataset)} frames; expected {plan.kept_frames}"
                )
        except Exception as exc:
            raise RuntimeError(
                f"repair was written but LeRobot validation failed: {exc}. "
                f"Original changed files are in {backup}"
            ) from exc
    return backup


def print_plan(plan: RepairPlan) -> None:
    print(f"dataset: {plan.root}")
    print(f"info.json reports: {plan.reported_episodes} episodes, {plan.reported_frames} frames")
    print(
        f"data parquet: {len(plan.data_episodes)} discovered episodes, "
        f"{plan.valid_data_prefix} valid contiguous episodes"
    )
    for key in sorted(plan.video_complete_prefix):
        print(
            f"video {key}: {plan.video_complete_prefix[key]} complete contiguous episodes "
            f"across {len(plan.video_files[key])} file(s)"
        )
    print(f"repair will keep: {plan.keep_episodes} episodes, {plan.kept_frames} frames")
    trailing = max(len(plan.data_episodes), plan.reported_episodes) - plan.keep_episodes
    print(f"trailing episodes to discard: {max(0, trailing)}")
    print("mode: read-only inspection")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Repair interrupted LeRobot v3 metadata and discard only an incomplete tail"
    )
    parser.add_argument("dataset_root", help="Local LeRobot dataset directory")
    parser.add_argument(
        "--apply",
        action="store_true",
        help="Create a backup and apply the printed repair plan",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Do not ask for interactive confirmation (requires --apply)",
    )
    parser.add_argument("--ffmpeg", default="ffmpeg", help="ffmpeg executable")
    parser.add_argument("--ffprobe", default="ffprobe", help="ffprobe executable")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.yes and not args.apply:
        print("--yes requires --apply", file=sys.stderr)
        return 2
    try:
        plan = build_repair_plan(
            args.dataset_root,
            video_probe=lambda path: probe_video(path, args.ffprobe),
        )
        print_plan(plan)
        if not args.apply:
            print("No files changed. Re-run with --apply after reviewing this plan.")
            return 0
        if not args.yes:
            confirmation = input(
                f"Type {plan.root.name} to create a backup and apply this repair: "
            ).strip()
            if confirmation != plan.root.name:
                print("Confirmation did not match; no files changed.")
                return 1
        backup = apply_repair(plan, ffmpeg=args.ffmpeg, ffprobe=args.ffprobe)
    except Exception as exc:
        print(f"Dataset repair failed: {exc}", file=sys.stderr)
        return 1

    print(f"repair_backup={backup}")
    print("repair_status=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
