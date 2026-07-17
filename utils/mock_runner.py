from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Any

from .io_utils import load_config, read_jsonl, write_jsonl
from .math_utils import wrap_angle


def _estimate_position(observation: dict[str, Any], metadata: dict[str, Any], geometry: dict[str, Any]) -> list[float] | None:
    """Crude pinhole estimate using observations only; never uses Ground Truth."""
    bbox = observation["bbox_xywh"]
    pixel_width = float(bbox[2])
    if pixel_width <= 1e-6:
        return None
    matrix = metadata["camera"]["camera_matrix"]
    fx, fy = float(matrix[0][0]), float(matrix[1][1])
    cx, cy = float(matrix[0][2]), float(matrix[1][2])
    armor_width = float(geometry[observation["armor_type"]]["width"])
    depth_x = fx * armor_width / pixel_width
    center_u = float(bbox[0]) + pixel_width / 2.0
    center_v = float(bbox[1]) + float(bbox[3]) / 2.0
    return [depth_x, -(center_u - cx) * depth_x / fx, -(center_v - cy) * depth_x / fy]


def run_mock(algorithm_input_dir: Path, run_dir: Path, config: dict[str, Any]) -> None:
    """Observation-only stub for validating plumbing without compiling C++.

    This is intentionally not an algorithm benchmark. It has access only to the
    isolated public input directory and cannot open evaluator Ground Truth.
    """
    metadata = load_config(algorithm_input_dir / "metadata.yaml")
    frames = read_jsonl(algorithm_input_dir / "frames.jsonl")
    observations = {
        int(row["frame_id"]): row
        for row in read_jsonl(algorithm_input_dir / "observations.jsonl")
    }
    mock = config["runner"].get("mock", {})
    warmup = int(mock.get("warmup_frames", 5))
    fire_interval = int(mock.get("fire_interval_frames", 12))
    muzzle_delay_ns = int(round(float(mock.get("muzzle_delay_s", 0.03)) * 1e9))
    outputs: list[dict[str, Any]] = []
    solver_outputs: list[dict[str, Any]] = []
    shots: list[dict[str, Any]] = []
    previous_position: list[float] | None = None
    previous_timestamp_ns: int | None = None
    previous_command_yaw = 0.0

    for frame in frames:
        begin = time.perf_counter()
        frame_id = int(frame["frame_id"])
        timestamp_ns = int(frame["timestamp_ns"])
        public_observations = [
            row for row in observations.get(frame_id, {"armors": []})["armors"]
            if row.get("valid", True)
        ]
        estimates: list[tuple[dict[str, Any], list[float]]] = []
        for observation in public_observations:
            position = _estimate_position(observation, metadata, config["armor_geometry"])
            if position is None:
                continue
            estimates.append((observation, position))
            solver_outputs.append(
                {
                    "frame_id": frame_id,
                    "timestamp_ns": timestamp_ns,
                    "observation_id": int(observation["observation_id"]),
                    # Association labels are copied only to output for Evaluator.
                    "target_hint_id": observation.get("target_hint_id"),
                    "armor_hint_id": observation.get("armor_hint_id"),
                    "position": position,
                    "yaw": 0.0,
                    "valid": True,
                }
            )

        estimates.sort(key=lambda item: int(item[0]["observation_id"]))
        chosen_observation, position = estimates[0] if estimates else (None, None)
        velocity = [0.0, 0.0, 0.0]
        if position is not None and previous_position is not None and previous_timestamp_ns is not None:
            dt = (timestamp_ns - previous_timestamp_ns) / 1e9
            if dt > 0.0:
                velocity = [(position[i] - previous_position[i]) / dt for i in range(3)]
        if position is not None:
            previous_position = position
            previous_timestamp_ns = timestamp_ns

        tracker_state = "lost" if position is None else ("detecting" if frame_id < warmup else "tracking")
        has_target = position is not None
        ekf_state = None
        aim: dict[str, Any] = {
            "valid": False, "aim_xyza": None, "target_id": None,
            "armor_id": None, "impact_timestamp_ns": None,
        }
        command = {"control": False, "shoot": False, "yaw": 0.0, "pitch": 0.0}
        if position is not None and chosen_observation is not None:
            ekf_state = [
                position[0], velocity[0], position[1], velocity[1], position[2], velocity[2],
                0.0, 0.0, 0.2, 0.0, 0.0,
            ]
            distance = math.sqrt(sum(value * value for value in position))
            flight_s = distance / float(frame["bullet_speed"])
            impact_timestamp_ns = timestamp_ns + muzzle_delay_ns + int(round(flight_s * 1e9))
            aim_position = [position[i] + velocity[i] * (flight_s + muzzle_delay_ns / 1e9) for i in range(3)]
            yaw = math.atan2(aim_position[1], aim_position[0])
            pitch = -math.atan2(aim_position[2], math.hypot(aim_position[0], aim_position[1]))
            target_hint = chosen_observation.get("target_hint_id")
            armor_hint = chosen_observation.get("armor_hint_id")
            aim = {
                "valid": True, "aim_xyza": aim_position + [0.0],
                "target_id": target_hint, "armor_id": armor_hint,
                "impact_timestamp_ns": impact_timestamp_ns,
            }
            stable = abs(wrap_angle(yaw - previous_command_yaw)) < math.radians(3.0)
            command = {
                "control": True,
                "shoot": tracker_state == "tracking" and stable and frame_id % fire_interval == 0,
                "yaw": yaw,
                "pitch": pitch,
            }
            previous_command_yaw = yaw
            if command["shoot"]:
                shots.append(
                    {
                        "shot_id": len(shots), "frame_id": frame_id,
                        "command_timestamp_ns": timestamp_ns,
                        "muzzle_timestamp_ns": timestamp_ns + muzzle_delay_ns,
                        "impact_timestamp_ns": impact_timestamp_ns,
                        "bullet_speed": float(frame["bullet_speed"]),
                        "command_yaw": yaw, "command_pitch": pitch,
                        "target_id": target_hint, "intended_armor_id": armor_hint,
                        "hit": False, "hit_armor_id": None,
                        "miss_distance": 1e9, "model": "pending_python_hit_check",
                    }
                )

        elapsed_ms = (time.perf_counter() - begin) * 1000.0
        outputs.append(
            {
                "frame_id": frame_id, "timestamp_ns": timestamp_ns,
                "tracker": {
                    "state": tracker_state, "has_target": has_target,
                    "last_armor_id": aim["armor_id"], "ekf_state": ekf_state,
                    "nis": None, "nees_internal": None,
                },
                "aimer": aim, "command": command,
                "timing_ms": {
                    "solver_tracker": elapsed_ms * 0.7, "aimer": elapsed_ms * 0.2,
                    "shooter": elapsed_ms * 0.1, "total": elapsed_ms,
                },
                "backend": "mock_observation_only",
            }
        )

    write_jsonl(run_dir / "algorithm_output.jsonl", outputs)
    write_jsonl(run_dir / "solver_output.jsonl", solver_outputs)
    write_jsonl(run_dir / "shots.jsonl", shots)
