from __future__ import annotations

from pathlib import Path
from typing import Any

from .io_utils import load_config, read_jsonl


class SchemaError(ValueError):
    pass


def _require(row: dict[str, Any], fields: tuple[str, ...], source: str) -> None:
    missing = [field for field in fields if field not in row]
    if missing:
        raise SchemaError(f"{source}: missing fields {missing}")


def validate_dataset(dataset_dir: Path) -> dict[str, int]:
    metadata = load_config(dataset_dir / "metadata.yaml")
    if metadata.get("schema_version") != "1.0":
        raise SchemaError(f"unsupported schema_version: {metadata.get('schema_version')}")
    frames = read_jsonl(dataset_dir / "frames.jsonl")
    observations = read_jsonl(dataset_dir / "observations.jsonl")
    targets = read_jsonl(dataset_dir / "ground_truth" / "target_states.jsonl")
    armors = read_jsonl(dataset_dir / "ground_truth" / "armor_states.jsonl")
    if not frames:
        raise SchemaError("frames.jsonl must not be empty")
    if len(observations) != len(frames):
        raise SchemaError("observations.jsonl must contain exactly one row per frame")
    previous_timestamp: int | None = None
    frame_ids: set[int] = set()
    timestamps: set[int] = set()
    for index, row in enumerate(frames):
        _require(row, ("frame_id", "timestamp_ns", "imu_q_wxyz", "bullet_speed", "valid"), f"frames.jsonl:{index + 1}")
        frame_id, timestamp = int(row["frame_id"]), int(row["timestamp_ns"])
        if frame_id in frame_ids:
            raise SchemaError(f"duplicate frame_id: {frame_id}")
        if previous_timestamp is not None and timestamp <= previous_timestamp:
            raise SchemaError("frame timestamps must be strictly increasing")
        if len(row["imu_q_wxyz"]) != 4:
            raise SchemaError(f"frame {frame_id}: imu_q_wxyz must have four elements")
        frame_ids.add(frame_id)
        timestamps.add(timestamp)
        previous_timestamp = timestamp
    for index, row in enumerate(observations):
        _require(row, ("frame_id", "timestamp_ns", "armors"), f"observations.jsonl:{index + 1}")
        if int(row["frame_id"]) not in frame_ids or int(row["timestamp_ns"]) not in timestamps:
            raise SchemaError(f"observation row {index + 1} does not match a frame")
        for armor in row["armors"]:
            _require(armor, ("observation_id", "class_id", "confidence", "corners_px", "bbox_xywh", "valid"), f"observations.jsonl:{index + 1}/armor")
            if len(armor["corners_px"]) != 4 or any(len(point) != 2 for point in armor["corners_px"]):
                raise SchemaError("corners_px must be a 4x2 array")
    for index, row in enumerate(targets):
        _require(row, ("timestamp_ns", "target_id", "position", "velocity", "yaw", "yaw_rate", "radius", "armor_count", "valid"), f"target_states.jsonl:{index + 1}")
    for index, row in enumerate(armors):
        _require(row, ("timestamp_ns", "target_id", "armor_id", "position", "yaw", "armor_type", "visible", "attackable", "valid"), f"armor_states.jsonl:{index + 1}")
    return {"frames": len(frames), "observations": len(observations), "target_states": len(targets), "armor_states": len(armors)}

