from __future__ import annotations

import math
from typing import Any


def _acceleration(velocity: list[float], physics: dict[str, Any]) -> list[float]:
    wind = [float(v) for v in physics.get("wind_velocity", [0.0, 0.0, 0.0])]
    relative = [velocity[i] - wind[i] for i in range(3)]
    speed = math.sqrt(sum(v * v for v in relative))
    drag = float(physics.get("drag_coefficient", 0.0))
    gravity = float(physics.get("gravity", 9.80665))
    return [
        -drag * speed * relative[0],
        -drag * speed * relative[1],
        -gravity - drag * speed * relative[2],
    ]


def _derivative(state: list[float], physics: dict[str, Any]) -> list[float]:
    acceleration = _acceleration(state[3:6], physics)
    return [state[3], state[4], state[5], *acceleration]


def rk4_step(state: list[float], dt: float, physics: dict[str, Any]) -> list[float]:
    k1 = _derivative(state, physics)
    s2 = [state[i] + 0.5 * dt * k1[i] for i in range(6)]
    k2 = _derivative(s2, physics)
    s3 = [state[i] + 0.5 * dt * k2[i] for i in range(6)]
    k3 = _derivative(s3, physics)
    s4 = [state[i] + dt * k3[i] for i in range(6)]
    k4 = _derivative(s4, physics)
    return [state[i] + dt * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) / 6.0 for i in range(6)]


def initial_state(
    muzzle_position: list[float], yaw: float, command_pitch: float, bullet_speed: float
) -> list[float]:
    elevation = -command_pitch  # Existing command convention: upward is negative.
    horizontal_speed = bullet_speed * math.cos(elevation)
    return [
        *[float(v) for v in muzzle_position],
        horizontal_speed * math.cos(yaw),
        horizontal_speed * math.sin(yaw),
        bullet_speed * math.sin(elevation),
    ]


def _height_at_horizontal_range(
    target_position: list[float], elevation: float, bullet_speed: float,
    muzzle_position: list[float], physics: dict[str, Any]
) -> tuple[float, float] | None:
    dx = float(target_position[0]) - float(muzzle_position[0])
    dy = float(target_position[1]) - float(muzzle_position[1])
    horizontal_range = math.hypot(dx, dy)
    if horizontal_range <= 1e-9:
        return None
    yaw = math.atan2(dy, dx)
    state = initial_state(muzzle_position, yaw, -elevation, bullet_speed)
    dt = float(physics.get("integration_dt_s", 0.002))
    max_time = float(physics.get("max_flight_time_s", 2.0))
    previous = state
    previous_range = 0.0
    elapsed = 0.0
    while elapsed < max_time:
        state = rk4_step(state, dt, physics)
        elapsed += dt
        current_range = math.hypot(state[0] - muzzle_position[0], state[1] - muzzle_position[1])
        if current_range >= horizontal_range:
            denominator = current_range - previous_range
            ratio = (horizontal_range - previous_range) / denominator if denominator > 1e-12 else 1.0
            z = previous[2] + ratio * (state[2] - previous[2])
            return z, elapsed - dt + ratio * dt
        previous = state
        previous_range = current_range
    return None


def solve_ideal_ballistic_pitch(
    target_position: list[float], bullet_speed: float, muzzle_position: list[float],
    physics: dict[str, Any]
) -> tuple[float, float] | None:
    """Independent numerical reference; returns (command_pitch, flight_time)."""
    target_z = float(target_position[2])

    def error(elevation: float) -> tuple[float, float] | None:
        result = _height_at_horizontal_range(
            target_position, elevation, bullet_speed, muzzle_position, physics
        )
        return None if result is None else (result[0] - target_z, result[1])

    low, high = math.radians(-30.0), math.radians(60.0)
    low_result, high_result = error(low), error(high)
    if low_result is None or high_result is None or low_result[0] * high_result[0] > 0.0:
        return None
    flight_time = 0.0
    for _ in range(32):
        middle = (low + high) / 2.0
        middle_result = error(middle)
        if middle_result is None:
            return None
        flight_time = middle_result[1]
        if abs(middle_result[0]) < 1e-6:
            return -middle, flight_time
        if low_result[0] * middle_result[0] <= 0.0:
            high = middle
        else:
            low = middle
            low_result = middle_result
    return -(low + high) / 2.0, flight_time

