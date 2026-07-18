from __future__ import annotations

import json
import math
import random
from pathlib import Path
from typing import Any

from . import SCHEMA_VERSION
from .io_utils import dump_json, write_jsonl
from .math_utils import wrap_angle


def _target_at(scene: dict[str, Any], t: float) -> dict[str, Any]:
    target = scene["target"]
    initial = [float(v) for v in target["initial_position"]]
    velocity = [float(v) for v in target["velocity"]]
    acceleration = [float(v) for v in target.get("acceleration", [0.0, 0.0, 0.0])]
    position = [initial[i] + velocity[i] * t + 0.5 * acceleration[i] * t * t for i in range(3)]
    current_velocity = [velocity[i] + acceleration[i] * t for i in range(3)]
    yaw_rate = float(target.get("yaw_rate", 0.0))
    yaw = wrap_angle(float(target.get("initial_yaw", 0.0)) + yaw_rate * t)
    return {
        "position": position,
        "velocity": current_velocity,
        "yaw": yaw,
        "yaw_rate": yaw_rate,
        "radius": float(target.get("radius", 0.2)),
        "radius_delta": float(target.get("radius_delta", 0.0)),
        "height_delta": float(target.get("height_delta", 0.0)),
        "armor_count": int(target.get("armor_count", 4)),
    }


def _armor_states(target: dict[str, Any], timestamp_ns: int, target_id: int, scene: dict[str, Any]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    count = int(target["armor_count"])
    center = target["position"]
    camera_bearing = math.atan2(center[1], center[0])
    for armor_id in range(count):
        angle = wrap_angle(float(target["yaw"]) + armor_id * 2.0 * math.pi / count)
        radius = float(target["radius"])
        if count == 4 and armor_id % 2 == 1:
            radius += float(target["radius_delta"])
        z = float(center[2]) + (float(target["height_delta"]) if armor_id % 2 else 0.0)
        position = [
            float(center[0]) - radius * math.cos(angle),
            float(center[1]) - radius * math.sin(angle),
            z,
        ]
        facing_error = abs(wrap_angle(angle - camera_bearing))
        visible = facing_error <= math.radians(float(scene.get("visibility_half_angle_deg", 75.0)))
        attackable = facing_error <= math.radians(float(scene.get("attackable_half_angle_deg", 60.0)))
        rows.append(
            {
                "timestamp_ns": timestamp_ns,
                "target_id": target_id,
                "armor_id": armor_id,
                "position": position,
                "yaw": angle,
                "armor_type": scene["target"].get("armor_type", "small"),
                "name": scene["target"].get("name", "two"),
                "visible": visible,
                "attackable": attackable,
                "valid": True,
            }
        )
    return rows


def _matrix3(values: list[float]) -> list[list[float]]:
    return [[float(values[row * 3 + column]) for column in range(3)] for row in range(3)]


def _transpose(matrix: list[list[float]]) -> list[list[float]]:
    return [[matrix[column][row] for column in range(3)] for row in range(3)]


def _matmul(left: list[list[float]], right: list[list[float]]) -> list[list[float]]:
    return [[sum(left[row][k] * right[k][column] for k in range(3)) for column in range(3)] for row in range(3)]


def _matvec(matrix: list[list[float]], vector: list[float]) -> list[float]:
    return [sum(matrix[row][column] * vector[column] for column in range(3)) for row in range(3)]


def _quaternion_matrix(q_wxyz: list[float]) -> list[list[float]]:
    w, x, y, z = [float(value) for value in q_wxyz]
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 1e-12:
        raise ValueError("IMU quaternion must be non-zero")
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def _project_armor(
    armor: dict[str, Any], camera: dict[str, Any], geometry: dict[str, Any],
    imu_q_wxyz: list[float]
) -> list[list[float]] | None:
    # This is the independent forward equivalent of Solver::reproject_armor().
    r_gimbal2imubody = _matrix3(camera["R_gimbal2imubody"])
    r_imubody2imuabs = _quaternion_matrix(imu_q_wxyz)
    r_gimbal2world = _matmul(
        _matmul(_transpose(r_gimbal2imubody), r_imubody2imuabs),
        r_gimbal2imubody,
    )
    r_camera2gimbal = _matrix3(camera["R_camera2gimbal"])
    r_world2gimbal = _transpose(r_gimbal2world)
    r_gimbal2camera = _transpose(r_camera2gimbal)
    t_camera2gimbal = [float(value) for value in camera["t_camera2gimbal"]]

    matrix = camera["camera_matrix"]
    fx, fy = float(matrix[0][0]), float(matrix[1][1])
    cx, cy = float(matrix[0][2]), float(matrix[1][2])
    k1, k2, p1, p2, k3 = [float(value) for value in camera["distortion_coefficients"]]
    size = geometry[armor["armor_type"]]
    half_width, half_height = float(size["width"]) / 2.0, float(size["height"]) / 2.0
    yaw = float(armor["yaw"])
    pitch = math.radians(-15.0 if armor["name"] == "outpost" else 15.0)
    sin_yaw, cos_yaw = math.sin(yaw), math.cos(yaw)
    sin_pitch, cos_pitch = math.sin(pitch), math.cos(pitch)
    r_armor2world = [
        [cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch],
        [sin_yaw * cos_pitch, cos_yaw, sin_yaw * sin_pitch],
        [-sin_pitch, 0.0, cos_pitch],
    ]
    r_armor2camera = _matmul(_matmul(r_gimbal2camera, r_world2gimbal), r_armor2world)
    position_world = [float(value) for value in armor["position"]]
    position_gimbal = _matvec(r_world2gimbal, position_world)
    t_armor2camera = _matvec(
        r_gimbal2camera,
        [position_gimbal[i] - t_camera2gimbal[i] for i in range(3)],
    )
    object_points = [
        [0.0, half_width, half_height],
        [0.0, -half_width, half_height],
        [0.0, -half_width, -half_height],
        [0.0, half_width, -half_height],
    ]
    points: list[list[float]] = []
    for object_point in object_points:
        rotated = _matvec(r_armor2camera, object_point)
        camera_point = [rotated[i] + t_armor2camera[i] for i in range(3)]
        if camera_point[2] <= 0.05:
            return None
        x = camera_point[0] / camera_point[2]
        y = camera_point[1] / camera_point[2]
        r2 = x * x + y * y
        radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
        distorted_x = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
        distorted_y = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
        points.append([fx * distorted_x + cx, fy * distorted_y + cy])
    return points


def _bbox(points: list[list[float]]) -> list[float]:
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return [min(xs), min(ys), max(xs) - min(xs), max(ys) - min(ys)]


def generate_dataset(config: dict[str, Any], project_root: Path) -> Path:
    scene_path = (project_root / config["data"]["case_config"]).resolve()
    scene = json.loads(scene_path.read_text(encoding="utf-8"))
    dataset_dir = (project_root / config["data"]["dataset_root"] / scene["name"]).resolve()
    gt_dir = dataset_dir / "ground_truth"
    gt_dir.mkdir(parents=True, exist_ok=True)

    duration_ns = int(round(float(scene["duration_s"]) * 1_000_000_000))
    step_ns = int(round(float(scene["dt_s"]) * 1_000_000_000))
    start_ns = int(scene.get("start_timestamp_ns", 0))
    frame_count = duration_ns // step_ns + 1
    target_id = int(scene["target"].get("target_id", 1))
    camera = config["camera"]
    geometry = config["armor_geometry"]
    noise = scene.get("observation", {})
    rng = random.Random(int(config.get("random_seed", 2026)))
    imu_q_wxyz = [float(value) for value in scene.get("imu_q_wxyz", [1.0, 0.0, 0.0, 0.0])]

    frames: list[dict[str, Any]] = []
    targets: list[dict[str, Any]] = []
    armors: list[dict[str, Any]] = []
    observations: list[dict[str, Any]] = []
    ideal_observations: list[dict[str, Any]] = []
    for frame_id in range(frame_count):
        timestamp_ns = start_ns + frame_id * step_ns
        t = (timestamp_ns - start_ns) / 1_000_000_000.0
        target = _target_at(scene, t)
        target_row = {
            "timestamp_ns": timestamp_ns,
            "target_id": target_id,
            **target,
            "valid": True,
            "source": "generated",
            "confidence": 1.0,
        }
        armor_rows = _armor_states(target, timestamp_ns, target_id, scene)
        frames.append(
            {
                "frame_id": frame_id,
                "timestamp_ns": timestamp_ns,
                "image_path": None,
                "imu_q_wxyz": imu_q_wxyz,
                "bullet_speed": float(scene.get("bullet_speed", 27.0)),
                "valid": True,
            }
        )
        targets.append(target_row)
        armors.extend(armor_rows)
        observed: list[dict[str, Any]] = []
        ideal_observed: list[dict[str, Any]] = []
        for armor in armor_rows:
            if not armor["visible"]:
                continue
            points = _project_armor(armor, camera, geometry, imu_q_wxyz)
            if points is None:
                continue
            ideal_points = [[float(x), float(y)] for x, y in points]
            ideal_observed.append(
                {
                    "target_id": target_id,
                    "armor_id": armor["armor_id"],
                    "corners_px": ideal_points,
                    "bbox_xywh": _bbox(ideal_points),
                    "valid": True,
                }
            )
            if rng.random() < float(noise.get("drop_probability", 0.0)):
                continue
            pixel_std = float(noise.get("pixel_noise_std", 0.0))
            if pixel_std > 0.0:
                points = [[x + rng.gauss(0.0, pixel_std), y + rng.gauss(0.0, pixel_std)] for x, y in points]
            observed.append(
                {
                    "observation_id": len(observed),
                    "target_hint_id": target_id,
                    "armor_hint_id": armor["armor_id"],
                    "class_id": int(scene["target"].get("class_id", 6)),
                    "color": scene["target"].get("color", "blue"),
                    "name": armor["name"],
                    "armor_type": armor["armor_type"],
                    "confidence": float(noise.get("confidence", 1.0)),
                    "corners_px": points,
                    "bbox_xywh": _bbox(points),
                    "visible": True,
                    "valid": True,
                }
            )
        observations.append({"frame_id": frame_id, "timestamp_ns": timestamp_ns, "armors": observed})
        ideal_observations.append({"frame_id": frame_id, "timestamp_ns": timestamp_ns, "armors": ideal_observed})

    metadata = {
        "schema_version": SCHEMA_VERSION,
        "dataset": {"name": scene["name"], "source": "generated", "description": scene.get("description", "")},
        "units": {"timestamp": "ns", "position": "m", "velocity": "m/s", "angle": "rad", "angular_velocity": "rad/s", "pixel": "px", "bullet_speed": "m/s"},
        "coordinate_system": {"world": {"x": "forward", "y": "left", "z": "up", "handedness": "right"}, "command": {"yaw_positive": "left", "pitch_positive": "down"}, "quaternion_order": "wxyz", "quaternion_meaning": "gimbal_to_world", "armor_corner_order": ["left_top", "right_top", "right_bottom", "left_bottom"]},
        "camera": camera,
        "armor_geometry": geometry,
        "simulation": {
            "start_timestamp_ns": start_ns,
            "duration_ns": duration_ns,
            "step_ns": step_ns,
            "observation_noise": bool(float(noise.get("pixel_noise_std", 0.0)) or float(noise.get("drop_probability", 0.0))),
            "ballistics": config["simulation"]["ballistics"],
        },
    }
    dump_json(dataset_dir / "metadata.yaml", metadata)
    write_jsonl(dataset_dir / "frames.jsonl", frames)
    write_jsonl(dataset_dir / "observations.jsonl", observations)
    # Kept below ground_truth so the algorithm input preparation cannot expose
    # ideal pixels to either backend.  Evaluator uses it only after inference.
    write_jsonl(gt_dir / "ideal_observations.jsonl", ideal_observations)
    write_jsonl(gt_dir / "target_states.jsonl", targets)
    write_jsonl(gt_dir / "armor_states.jsonl", armors)
    write_jsonl(gt_dir / "gimbal_states.jsonl", [])
    write_jsonl(gt_dir / "shots.jsonl", [])
    return dataset_dir
