from __future__ import annotations

import csv
import html
from pathlib import Path
from typing import Any


def write_svg_plot(run_dir: Path) -> None:
    path = run_dir / "frame_errors.csv"
    points: list[tuple[int, float]] = []
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        for row in csv.DictReader(stream):
            if row.get("position_error"):
                points.append((int(row["frame_id"]), float(row["position_error"])))
    width, height, pad = 1000, 420, 45
    max_x = max((x for x, _ in points), default=1)
    max_y = max((y for _, y in points), default=1.0) or 1.0
    coords = " ".join(
        f"{pad + x / max_x * (width - 2 * pad):.2f},{height - pad - y / max_y * (height - 2 * pad):.2f}"
        for x, y in points
    )
    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
<rect width="100%" height="100%" fill="white"/><line x1="{pad}" y1="{height-pad}" x2="{width-pad}" y2="{height-pad}" stroke="#333"/>
<line x1="{pad}" y1="{pad}" x2="{pad}" y2="{height-pad}" stroke="#333"/><polyline fill="none" stroke="#2563eb" stroke-width="2" points="{coords}"/>
<text x="{width/2}" y="25" text-anchor="middle" font-family="sans-serif">Tracker position error (m)</text>
<text x="{width/2}" y="{height-8}" text-anchor="middle" font-family="sans-serif">frame_id</text><text x="8" y="20" font-family="sans-serif">max={max_y:.6f} m</text></svg>'''
    (run_dir / "position_error.svg").write_text(svg, encoding="utf-8")


def print_summary(summary: dict[str, Any], run_dir: Path) -> None:
    tracker_rmse = summary["tracker"]["position"]["rmse"]
    aim_rmse = summary["aimer"]["aim_point"]["rmse"]
    hit_rate = summary["hit"]["hit_rate"]
    print(f"run_dir: {run_dir}")
    print(f"backend: {summary['backend']}")
    print(f"frames: {summary['frames']['total']}, tracking ratio: {summary['frames']['tracking_ratio']:.3f}")
    print(f"tracker position RMSE: {tracker_rmse if tracker_rmse is not None else 'N/A'}")
    print(f"aimer point RMSE: {aim_rmse if aim_rmse is not None else 'N/A'}")
    print(
        "shoot-enabled/evaluable/hits/hit rate: "
        f"{summary['hit']['shots']}/{summary['hit'].get('evaluable_shots', 'N/A')}/"
        f"{summary['hit']['hits']}/{hit_rate if hit_rate is not None else 'N/A'}"
    )
