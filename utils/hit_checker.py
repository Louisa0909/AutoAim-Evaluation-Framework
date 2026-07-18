from __future__ import annotations

import math
from typing import Any

from .ballistics import initial_state, rk4_step
from .timeline import StateTimeline


def _plane_geometry(
    state: list[float], truth: dict[str, Any], geometry: dict[str, Any]
) -> tuple[float, float]:
    """Return signed plane distance and miss distance to the armor rectangle."""
    center = [float(v) for v in truth["position"]]
    yaw = float(truth["yaw"])
    delta = [state[i] - center[i] for i in range(3)]
    signed = math.cos(yaw) * delta[0] + math.sin(yaw) * delta[1]
    tangent = -math.sin(yaw) * delta[0] + math.cos(yaw) * delta[1]
    vertical = delta[2]
    size = geometry[truth["armor_type"]]
    outside_tangent = max(0.0, abs(tangent) - float(size["width"]) / 2.0)
    outside_vertical = max(0.0, abs(vertical) - float(size["height"]) / 2.0)
    return signed, math.hypot(outside_tangent, outside_vertical)


def simulate_physical_shot(
    command_yaw: float,
    command_pitch: float,
    bullet_speed: float,
    muzzle_timestamp_ns: int,
    target_id: int,
    armor_id: int,
    armor_timeline: StateTimeline,
    geometry: dict[str, Any],
    physics: dict[str, Any],
) -> dict[str, Any]:
    muzzle_position = [float(v) for v in physics.get("muzzle_position", [0.0, 0.0, 0.0])]
    state = initial_state(muzzle_position, command_yaw, command_pitch, bullet_speed)
    dt = float(physics.get("integration_dt_s", 0.002))
    max_time = float(physics.get("max_flight_time_s", 2.0))
    previous_signed: float | None = None
    previous_state: list[float] | None = None
    previous_elapsed: float | None = None
    best_miss = float("inf")
    elapsed = 0.0
    while elapsed <= max_time:
        timestamp_ns = muzzle_timestamp_ns + int(round(elapsed * 1e9))
        truth = armor_timeline.sample(timestamp_ns, target_id, armor_id)
        if truth is None:
            previous_signed = None
            previous_state = None
            previous_elapsed = None
            state = rk4_step(state, dt, physics)
            elapsed += dt
            continue
        signed, plane_miss = _plane_geometry(state, truth, geometry)
        if abs(signed) < 0.15:
            best_miss = min(best_miss, plane_miss)
        crossed = previous_signed is not None and previous_signed * signed <= 0.0
        if crossed and previous_state is not None and previous_elapsed is not None:
            denominator = previous_signed - signed
            ratio = previous_signed / denominator if abs(denominator) > 1e-12 else 1.0
            ratio = min(1.0, max(0.0, ratio))
            crossing_elapsed = previous_elapsed + ratio * (elapsed - previous_elapsed)
            crossing_timestamp_ns = muzzle_timestamp_ns + int(round(crossing_elapsed * 1e9))
            crossing_state = [
                previous_state[i] + ratio * (state[i] - previous_state[i]) for i in range(6)
            ]
            # StateTimeline performs position/yaw interpolation here, so the
            # bullet and moving armor are evaluated at the same crossing time.
            crossing_truth = armor_timeline.sample(crossing_timestamp_ns, target_id, armor_id)
            if crossing_truth is None:
                previous_signed = signed
                previous_state = list(state)
                previous_elapsed = elapsed
                state = rk4_step(state, dt, physics)
                elapsed += dt
                continue
            _, crossing_miss = _plane_geometry(crossing_state, crossing_truth, geometry)
            hit = crossing_miss <= 1e-9
            return {
                "evaluation_valid": True,
                "hit": hit,
                "miss_distance": crossing_miss,
                "impact_timestamp_ns": crossing_timestamp_ns,
                "impact_position": crossing_state[:3],
                "flight_time_s": crossing_elapsed,
            }
        previous_signed = signed
        previous_state = list(state)
        previous_elapsed = elapsed
        state = rk4_step(state, dt, physics)
        elapsed += dt
    return {
        "evaluation_valid": False,
        "hit": False,
        "miss_distance": best_miss,
        "impact_timestamp_ns": None,
        "impact_position": None,
        "flight_time_s": None,
    }
