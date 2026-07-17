from __future__ import annotations

import math
from pathlib import Path
from typing import Any

from . import SCHEMA_VERSION
from .ballistics import solve_ideal_ballistic_pitch
from .hit_checker import simulate_physical_shot
from .io_utils import dump_json, read_jsonl, write_csv, write_jsonl
from .math_utils import angle_error, distance3, percentile, rmse
from .timeline import StateTimeline


def _metric(values: list[float]) -> dict[str, Any]:
    return {
        "count": len(values),
        "rmse": rmse(values),
        "mean_abs": sum(abs(v) for v in values) / len(values) if values else None,
        "max_abs": max((abs(v) for v in values), default=None),
        "p95_abs": percentile([abs(v) for v in values], 0.95),
        "p99_abs": percentile([abs(v) for v in values], 0.99),
    }


def evaluate(dataset_dir: Path, run_dir: Path, config: dict[str, Any]) -> dict[str, Any]:
    target_rows = read_jsonl(dataset_dir / "ground_truth" / "target_states.jsonl")
    armor_rows = read_jsonl(dataset_dir / "ground_truth" / "armor_states.jsonl")
    frames = read_jsonl(dataset_dir / "frames.jsonl")
    frames_by_id = {int(frame["frame_id"]): frame for frame in frames}
    outputs = read_jsonl(run_dir / "algorithm_output.jsonl")
    solver_outputs = read_jsonl(run_dir / "solver_output.jsonl")
    shots = read_jsonl(run_dir / "shots.jsonl")
    target_timeline = StateTimeline(target_rows, ("target_id",))
    armor_timeline = StateTimeline(armor_rows, ("target_id", "armor_id"))
    armor_ids_by_target: dict[int, list[int]] = {}
    for armor in armor_rows:
        tid, aid = int(armor["target_id"]), int(armor["armor_id"])
        armor_ids_by_target.setdefault(tid, [])
        if aid not in armor_ids_by_target[tid]:
            armor_ids_by_target[tid].append(aid)
    target_id = int(target_rows[0]["target_id"]) if target_rows else 1
    warmup_frames = int(config["evaluation"].get("warmup_frames", 0))
    physics = config["simulation"]["ballistics"]
    muzzle_position = [float(v) for v in physics.get("muzzle_position", [0.0, 0.0, 0.0])]

    frame_errors: list[dict[str, Any]] = []
    position_errors: list[float] = []
    velocity_errors: list[float] = []
    yaw_errors: list[float] = []
    yaw_rate_errors: list[float] = []
    aim_errors: list[float] = []
    yaw_command_errors: list[float] = []
    los_pitch_differences: list[float] = []
    ballistic_pitch_errors: list[float] = []
    valid_tracking = 0
    first_tracking_timestamp: int | None = None
    total_latencies: list[float] = []
    output_by_frame: dict[int, dict[str, Any]] = {}
    shooter_tp = shooter_fp = shooter_fn = shooter_tn = 0
    for output in outputs:
        frame_id = int(output["frame_id"])
        output_by_frame[frame_id] = output
        timestamp_ns = int(output["timestamp_ns"])
        truth = target_timeline.sample(timestamp_ns, target_id)
        tracker = output["tracker"]
        state = tracker["state"]
        if state == "tracking":
            valid_tracking += 1
            if first_tracking_timestamp is None:
                first_tracking_timestamp = timestamp_ns
        total_latencies.append(float(output["timing_ms"]["total"]))
        row: dict[str, Any] = {
            "frame_id": frame_id,
            "timestamp_ns": timestamp_ns,
            "tracker_state": state,
            "tracker_valid": bool(tracker["has_target"]),
            "error_x": None, "error_y": None, "error_z": None,
            "position_error": None, "velocity_error": None,
            "yaw_error": None, "yaw_rate_error": None,
            "aim_point_error": None, "command_yaw_error": None,
            "aimer_internal_armor_id": None, "matched_gt_armor_id": None,
            "line_of_sight_pitch_difference": None,
            "ideal_ballistic_pitch": None, "ballistic_pitch_error": None,
        }
        state_vector = tracker.get("ekf_state")
        include_metric = frame_id >= warmup_frames and truth is not None and isinstance(state_vector, list) and len(state_vector) >= 11
        if include_metric:
            ex = float(state_vector[0]) - float(truth["position"][0])
            ey = float(state_vector[2]) - float(truth["position"][1])
            ez = float(state_vector[4]) - float(truth["position"][2])
            position_error = math.sqrt(ex * ex + ey * ey + ez * ez)
            velocity_error = distance3([state_vector[1], state_vector[3], state_vector[5]], truth["velocity"])
            yaw_error = angle_error(float(state_vector[6]), float(truth["yaw"]))
            yaw_rate_error = float(state_vector[7]) - float(truth["yaw_rate"])
            row.update({"error_x": ex, "error_y": ey, "error_z": ez, "position_error": position_error, "velocity_error": velocity_error, "yaw_error": yaw_error, "yaw_rate_error": yaw_rate_error})
            position_errors.append(position_error)
            velocity_errors.append(velocity_error)
            yaw_errors.append(yaw_error)
            yaw_rate_errors.append(yaw_rate_error)
        aimer = output.get("aimer", {})
        if aimer.get("valid") and aimer.get("aim_xyza") is not None and aimer.get("impact_timestamp_ns") is not None:
            # Tracker/Aimer armor indices are initialized from the first observation;
            # generated GT indices identify physical plates.  Do not assume those two
            # cyclic index spaces have the same phase.  Associate the selected point
            # to the nearest physical plate at the exact Aimer prediction timestamp.
            impact_timestamp = int(aimer["impact_timestamp_ns"])
            aimer_target_id = int(aimer["target_id"])
            candidates = [
                armor_timeline.sample(impact_timestamp, aimer_target_id, armor_id)
                for armor_id in armor_ids_by_target.get(aimer_target_id, [])
            ]
            candidates = [candidate for candidate in candidates if candidate is not None]
            armor_truth = min(
                candidates,
                key=lambda candidate: distance3(aimer["aim_xyza"], candidate["position"]),
                default=None,
            )
            if armor_truth is not None:
                aim_error = distance3(aimer["aim_xyza"], armor_truth["position"])
                ideal_yaw = math.atan2(float(armor_truth["position"][1]), float(armor_truth["position"][0]))
                # Existing io::Command/Aimer convention: looking upward is negative pitch.
                ideal_pitch = -math.atan2(float(armor_truth["position"][2]), math.hypot(float(armor_truth["position"][0]), float(armor_truth["position"][1])))
                yaw_error = angle_error(float(output["command"]["yaw"]), ideal_yaw)
                los_pitch_difference = angle_error(float(output["command"]["pitch"]), ideal_pitch)
                bullet_speed = float(frames_by_id[frame_id]["bullet_speed"])
                ballistic_solution = solve_ideal_ballistic_pitch(
                    [float(v) for v in armor_truth["position"]], bullet_speed,
                    muzzle_position, physics,
                )
                ideal_ballistic_pitch = ballistic_solution[0] if ballistic_solution else None
                ballistic_pitch_error = (
                    angle_error(float(output["command"]["pitch"]), ideal_ballistic_pitch)
                    if ideal_ballistic_pitch is not None else None
                )
                row.update({
                    "aim_point_error": aim_error,
                    "aimer_internal_armor_id": aimer.get("armor_id"),
                    "matched_gt_armor_id": armor_truth["armor_id"],
                    "command_yaw_error": yaw_error,
                    "line_of_sight_pitch_difference": los_pitch_difference,
                    "ideal_ballistic_pitch": ideal_ballistic_pitch,
                    "ballistic_pitch_error": ballistic_pitch_error,
                })
                if frame_id >= warmup_frames:
                    aim_errors.append(aim_error)
                    yaw_command_errors.append(yaw_error)
                    los_pitch_differences.append(los_pitch_difference)
                    if ballistic_pitch_error is not None:
                        ballistic_pitch_errors.append(ballistic_pitch_error)
                gt_should_shoot = bool(armor_truth.get("attackable", False)) and frame_id >= warmup_frames
                algorithm_shoot = bool(output.get("command", {}).get("shoot", False))
                if algorithm_shoot and gt_should_shoot:
                    shooter_tp += 1
                elif algorithm_shoot:
                    shooter_fp += 1
                elif gt_should_shoot:
                    shooter_fn += 1
                else:
                    shooter_tn += 1
        frame_errors.append(row)

    # Both backends only record fire events; Evaluator owns independent physics.
    for shot in shots:
        if shot.get("target_id") is None or shot.get("intended_armor_id") is None:
            continue
        output = output_by_frame.get(int(shot["frame_id"]), {})
        aimer = output.get("aimer", {})
        if aimer.get("aim_xyza") is not None and aimer.get("impact_timestamp_ns") is not None:
            candidates = [
                armor_timeline.sample(int(aimer["impact_timestamp_ns"]), int(shot["target_id"]), armor_id)
                for armor_id in armor_ids_by_target.get(int(shot["target_id"]), [])
            ]
            candidates = [candidate for candidate in candidates if candidate is not None]
            matched = min(candidates, key=lambda candidate: distance3(aimer["aim_xyza"], candidate["position"]), default=None)
            if matched is not None:
                shot["aimer_internal_armor_id"] = shot["intended_armor_id"]
                shot["intended_armor_id"] = int(matched["armor_id"])
        result = simulate_physical_shot(
            float(shot["command_yaw"]), float(shot["command_pitch"]),
            float(shot["bullet_speed"]), int(shot["muzzle_timestamp_ns"]),
            int(shot["target_id"]), int(shot["intended_armor_id"]),
            armor_timeline, config["armor_geometry"], physics,
        )
        shot["hit"] = result["hit"]
        shot["physical_evaluation_valid"] = result["evaluation_valid"]
        shot["hit_armor_id"] = shot["intended_armor_id"] if result["hit"] else None
        shot["miss_distance"] = result["miss_distance"]
        shot["evaluated_impact_timestamp_ns"] = result["impact_timestamp_ns"]
        shot["impact_position"] = result["impact_position"]
        shot["physical_flight_time_s"] = result["flight_time_s"]
        shot["model"] = "independent_rk4_configured_physics"
    write_jsonl(run_dir / "shots.jsonl", shots)

    solver_errors: list[dict[str, Any]] = []
    solver_position_errors: list[float] = []
    solver_yaw_errors: list[float] = []
    for output in solver_outputs:
        truth = armor_timeline.sample(int(output["timestamp_ns"]), int(output["target_hint_id"]), int(output["armor_hint_id"]))
        if truth is None or not output.get("valid", True):
            continue
        error = [float(output["position"][i]) - float(truth["position"][i]) for i in range(3)]
        position_error = math.sqrt(sum(v * v for v in error))
        yaw_error = angle_error(float(output["yaw"]), float(truth["yaw"]))
        solver_position_errors.append(position_error)
        solver_yaw_errors.append(yaw_error)
        solver_errors.append(
            {
                "frame_id": output["frame_id"], "timestamp_ns": output["timestamp_ns"],
                "observation_id": output["observation_id"], "target_id": output["target_hint_id"], "armor_id": output["armor_hint_id"],
                "error_x": error[0], "error_y": error[1], "error_z": error[2],
                "position_error": position_error, "yaw_error": yaw_error,
            }
        )

    write_csv(run_dir / "frame_errors.csv", ["frame_id", "timestamp_ns", "tracker_state", "tracker_valid", "error_x", "error_y", "error_z", "position_error", "velocity_error", "yaw_error", "yaw_rate_error", "aimer_internal_armor_id", "matched_gt_armor_id", "aim_point_error", "command_yaw_error", "line_of_sight_pitch_difference", "ideal_ballistic_pitch", "ballistic_pitch_error"], frame_errors)
    write_csv(run_dir / "solver_errors.csv", ["frame_id", "timestamp_ns", "observation_id", "target_id", "armor_id", "error_x", "error_y", "error_z", "position_error", "yaw_error"], solver_errors)

    evaluable_shots = [shot for shot in shots if shot.get("physical_evaluation_valid", False)]
    hits = [shot for shot in evaluable_shots if shot.get("hit")]
    miss_distances = [float(shot["miss_distance"]) for shot in evaluable_shots if math.isfinite(float(shot["miss_distance"]))]
    first_timestamp = int(outputs[0]["timestamp_ns"]) if outputs else 0
    summary = {
        "schema_version": SCHEMA_VERSION,
        "backend": outputs[0].get("backend", "unknown") if outputs else "unknown",
        "frames": {"total": len(outputs), "tracking": valid_tracking, "tracking_ratio": valid_tracking / len(outputs) if outputs else None},
        "solver": {"position": _metric(solver_position_errors), "yaw": _metric(solver_yaw_errors)},
        "tracker": {
            "position": _metric(position_errors), "velocity": _metric(velocity_errors),
            "yaw": _metric(yaw_errors), "yaw_rate": _metric(yaw_rate_errors),
            "convergence_time_s": (first_tracking_timestamp - first_timestamp) / 1e9 if first_tracking_timestamp is not None else None,
        },
        "aimer": {
            "aim_point": _metric(aim_errors),
            "command_yaw": _metric(yaw_command_errors),
            "line_of_sight_pitch_difference": _metric(los_pitch_differences),
            "ballistic_pitch_error": _metric(ballistic_pitch_errors),
            "pitch_metric_note": "LOS difference is diagnostic, not an algorithm error; ballistic_pitch_error is the physical reference metric.",
        },
        "shooter": {
            "shots": len(shots), "true_positive": shooter_tp, "false_positive": shooter_fp,
            "false_negative": shooter_fn, "true_negative": shooter_tn,
            "precision": shooter_tp / (shooter_tp + shooter_fp) if shooter_tp + shooter_fp else None,
            "recall": shooter_tp / (shooter_tp + shooter_fn) if shooter_tp + shooter_fn else None,
        },
        "hit": {
            "shots": len(shots), "evaluable_shots": len(evaluable_shots),
            "unevaluable_shots": len(shots) - len(evaluable_shots), "hits": len(hits),
            "hit_rate": len(hits) / len(evaluable_shots) if evaluable_shots else None,
            "physical_miss_distance": _metric(miss_distances),
            "model": "independent_rk4_configured_physics", "physics": physics,
            "gimbal_assumption": config["simulation"]["gimbal_model"],
        },
        "latency_ms": _metric(total_latencies),
        "evaluation": {"warmup_frames_excluded": warmup_frames},
    }
    dump_json(run_dir / "summary.json", summary)
    return summary
