from __future__ import annotations

from bisect import bisect_left
from collections import defaultdict
from typing import Any

from .math_utils import lerp, lerp_angle


class StateTimeline:
    def __init__(self, rows: list[dict[str, Any]], id_fields: tuple[str, ...]):
        grouped: dict[tuple[Any, ...], list[dict[str, Any]]] = defaultdict(list)
        for row in rows:
            grouped[tuple(row[field] for field in id_fields)].append(row)
        self._rows = {key: sorted(value, key=lambda row: int(row["timestamp_ns"])) for key, value in grouped.items()}
        self._times = {key: [int(row["timestamp_ns"]) for row in value] for key, value in self._rows.items()}

    def sample(self, timestamp_ns: int, *ids: Any) -> dict[str, Any] | None:
        key = tuple(ids)
        rows = self._rows.get(key)
        if not rows:
            return None
        times = self._times[key]
        index = bisect_left(times, timestamp_ns)
        if index < len(times) and times[index] == timestamp_ns:
            return dict(rows[index])
        if index == 0 or index >= len(times):
            return None
        left, right = rows[index - 1], rows[index]
        ratio = (timestamp_ns - int(left["timestamp_ns"])) / (int(right["timestamp_ns"]) - int(left["timestamp_ns"]))
        result = dict(left)
        result["timestamp_ns"] = timestamp_ns
        for field in ("position", "velocity"):
            if field in left and field in right:
                result[field] = [lerp(float(a), float(b), ratio) for a, b in zip(left[field], right[field])]
        for field in ("yaw",):
            if field in left and field in right:
                result[field] = lerp_angle(float(left[field]), float(right[field]), ratio)
        for field in ("yaw_rate", "radius", "radius_delta", "height_delta"):
            if field in left and field in right:
                result[field] = lerp(float(left[field]), float(right[field]), ratio)
        return result

    def keys(self) -> list[tuple[Any, ...]]:
        return list(self._rows)

