from __future__ import annotations

import math
from typing import Any

from .ballistics import initial_state, rk4_step
from .timeline import StateTimeline


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
    best_miss = float("inf")
    elapsed = 0.0
    while elapsed <= max_time:
        timestamp_ns = muzzle_timestamp_ns + int(round(elapsed * 1e9))
        truth = armor_timeline.sample(timestamp_ns, target_id, armor_id)
        if truth is None:
            state = rk4_step(state, dt, physics)
            elapsed += dt
            continue
        center = [float(v) for v in truth["position"]]
        yaw = float(truth["yaw"])
        delta = [state[i] - center[i] for i in range(3)]
        signed = math.cos(yaw) * delta[0] + math.sin(yaw) * delta[1]
        tangent = -math.sin(yaw) * delta[0] + math.cos(yaw) * delta[1]
        vertical = delta[2]
        size = geometry[truth["armor_type"]]
        outside_tangent = max(0.0, abs(tangent) - float(size["width"]) / 2.0)
        outside_vertical = max(0.0, abs(vertical) - float(size["height"]) / 2.0)
        plane_miss = math.hypot(outside_tangent, outside_vertical)
        if abs(signed) < 0.15:
            best_miss = min(best_miss, plane_miss)
        crossed = previous_signed is not None and previous_signed * signed <= 0.0
        if crossed:
            hit = plane_miss <= 1e-9
            return {
                "evaluation_valid": True,
                "hit": hit,
                "miss_distance": plane_miss,
                "impact_timestamp_ns": timestamp_ns,
                "impact_position": state[:3],
                "flight_time_s": elapsed,
            }
        previous_signed = signed
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
