from __future__ import annotations

import math
from typing import Sequence


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def angle_error(predicted: float, truth: float) -> float:
    return wrap_angle(predicted - truth)


def norm3(value: Sequence[float]) -> float:
    return math.sqrt(sum(float(x) * float(x) for x in value[:3]))


def distance3(a: Sequence[float], b: Sequence[float]) -> float:
    return norm3([float(a[i]) - float(b[i]) for i in range(3)])


def percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * q
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def rmse(values: list[float]) -> float | None:
    if not values:
        return None
    return math.sqrt(sum(value * value for value in values) / len(values))


def lerp(a: float, b: float, ratio: float) -> float:
    return a + (b - a) * ratio


def lerp_angle(a: float, b: float, ratio: float) -> float:
    return wrap_angle(a + wrap_angle(b - a) * ratio)

