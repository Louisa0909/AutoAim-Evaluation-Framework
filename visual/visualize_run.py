"""Visualize one complete offline-test run.

Set RUN_DIR below to the run_* directory that you want to inspect, then run:
    python offline_test/visual/visualize_run.py

Images and README.md are written to RUN_DIR/visual_report/.
"""
from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path
from typing import Any, Iterable

# ---------------------------------------------------------------------------
# EDIT THIS VALUE: it is intentionally a concrete run directory, not "latest".
# This makes every report reproducible and prevents silently plotting a new run.
# ---------------------------------------------------------------------------
RUN_DIR = Path(r"/home/xiaoyu/EGA/offline_test/output/run_20260715_115108_064823_spinning_target")

try:
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MaxNLocator
except ImportError as exc:
    print("Missing visualization dependency: matplotlib", file=sys.stderr)
    print("Install it with: python -m pip install -r offline_test/visual/requirements.txt", file=sys.stderr)
    raise SystemExit(2) from exc


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def read_csv(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def number(value: Any) -> float | None:
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (TypeError, ValueError):
        return None


def series(rows: Iterable[dict[str, Any]], key: str) -> tuple[list[float], list[float]]:
    xs, ys = [], []
    for row in rows:
        x, y = number(row.get("timestamp_ns")), number(row.get(key))
        if x is not None and y is not None:
            xs.append(x / 1e9)
            ys.append(y)
    return xs, ys


def nested_series(outputs: Iterable[dict[str, Any]], keys: tuple[str, ...]) -> tuple[list[float], list[float]]:
    xs, ys = [], []
    for row in outputs:
        value: Any = row
        for key in keys:
            value = value.get(key) if isinstance(value, dict) else None
        x, y = number(row.get("timestamp_ns")), number(value)
        if x is not None and y is not None:
            xs.append(x / 1e9)
            ys.append(y)
    return xs, ys


def style_axis(ax: Any, title: str, ylabel: str) -> None:
    ax.set_title(title)
    ax.set_xlabel("Time since dataset start (s)")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    ax.xaxis.set_major_locator(MaxNLocator(8))


def plot_csv_lines(ax: Any, rows: list[dict[str, str]], specs: list[tuple[str, str]], title: str, ylabel: str) -> None:
    for key, label in specs:
        x, y = series(rows, key)
        if y:
            ax.plot(x, y, linewidth=1.25, label=label)
    style_axis(ax, title, ylabel)
    if ax.lines:
        ax.legend(fontsize=8)


def save(fig: Any, output: Path, name: str) -> None:
    fig.tight_layout()
    fig.savefig(output / name, dpi=180, bbox_inches="tight")
    plt.close(fig)


def metric(summary: dict[str, Any], *keys: str) -> Any:
    value: Any = summary
    for key in keys:
        value = value.get(key) if isinstance(value, dict) else None
    return value


def main() -> int:
    run_dir = RUN_DIR.resolve()
    required = ["summary.json", "frame_errors.csv", "algorithm_output.jsonl"]
    missing = [name for name in required if not (run_dir / name).exists()]
    if missing:
        raise FileNotFoundError(f"RUN_DIR is not a valid completed run: {run_dir}; missing: {missing}")

    out = run_dir / "visual_report"
    out.mkdir(exist_ok=True)
    summary = read_json(run_dir / "summary.json")
    frames = read_csv(run_dir / "frame_errors.csv")
    solver = read_csv(run_dir / "solver_errors.csv")
    outputs = read_jsonl(run_dir / "algorithm_output.jsonl")
    shots = read_jsonl(run_dir / "shots.jsonl")

    # 01: error overview. Every y value is an absolute/distance error; lower is better.
    fig, axes = plt.subplots(2, 2, figsize=(14, 8))
    plot_csv_lines(axes[0, 0], solver, [("position_error", "Solver position")], "Solver 3D position error", "Euclidean error (m)")
    plot_csv_lines(axes[0, 1], solver, [("yaw_error", "Solver yaw")], "Solver armor yaw error", "Wrapped angular error (rad)")
    plot_csv_lines(axes[1, 0], frames, [("position_error", "Position"), ("velocity_error", "Velocity")], "Tracker center-state errors", "Position (m) / velocity (m/s)")
    plot_csv_lines(axes[1, 1], frames, [("yaw_error", "Yaw"), ("yaw_rate_error", "Yaw rate")], "Tracker rotation-state errors", "Yaw (rad) / yaw rate (rad/s)")
    save(fig, out, "01_solver_tracker_errors.png")

    # 02: Aimer metrics and plate association diagnostics.
    fig, axes = plt.subplots(2, 2, figsize=(14, 8))
    plot_csv_lines(axes[0, 0], frames, [("aim_point_error", "Aim point")], "Aimer point vs matched physical armor", "3D distance error (m)")
    plot_csv_lines(axes[0, 1], frames, [("command_yaw_error", "Command yaw"), ("ballistic_pitch_error", "Ballistic pitch")], "Command-angle errors", "Wrapped angular error (rad)")
    plot_csv_lines(axes[1, 0], frames, [("line_of_sight_pitch_difference", "Pitch vs LOS")], "Pitch lead caused by ballistics (diagnostic)", "Command pitch - LOS pitch (rad)")
    plot_csv_lines(axes[1, 1], frames, [("aimer_internal_armor_id", "Aimer internal ID"), ("matched_gt_armor_id", "Matched GT physical ID")], "Armor association audit", "Armor index (integer)")
    axes[1, 1].yaxis.set_major_locator(MaxNLocator(integer=True))
    save(fig, out, "02_aimer_and_association.png")

    # 03: raw EKF state estimates. These are estimates, not errors.
    state_specs = [
        (("tracker", "ekf_state"), 0, "center x", "m"), (("tracker", "ekf_state"), 2, "center y", "m"),
        (("tracker", "ekf_state"), 4, "center z", "m"), (("tracker", "ekf_state"), 6, "yaw", "rad"),
        (("tracker", "ekf_state"), 7, "yaw rate", "rad/s"), (("tracker", "ekf_state"), 8, "radius 1", "m"),
        (("tracker", "ekf_state"), 9, "radius 2", "m"), (("tracker", "ekf_state"), 10, "height delta", "m"),
    ]
    fig, axes = plt.subplots(4, 2, figsize=(14, 13))
    for ax, (prefix, index, label, unit) in zip(axes.flat, state_specs):
        xs, ys = [], []
        for row in outputs:
            values = row.get(prefix[0], {}).get(prefix[1])
            if isinstance(values, list) and len(values) > index:
                value = number(values[index])
                if value is not None:
                    xs.append(float(row["timestamp_ns"]) / 1e9); ys.append(value)
        ax.plot(xs, ys, linewidth=1.2)
        style_axis(ax, f"EKF estimate: {label}", f"{label} ({unit})")
    save(fig, out, "03_tracker_ekf_states.png")

    # 04: commands, target point, and execution latency.
    fig, axes = plt.subplots(2, 2, figsize=(14, 8))
    for keys, label in [(('command', 'yaw'), 'yaw'), (('command', 'pitch'), 'pitch')]:
        x, y = nested_series(outputs, keys); axes[0, 0].plot(x, y, label=label)
    style_axis(axes[0, 0], "Gimbal commands", "Command angle (rad)"); axes[0, 0].legend()
    for index, label in enumerate(("aim x", "aim y", "aim z")):
        xs, ys = [], []
        for row in outputs:
            point = row.get("aimer", {}).get("aim_xyza")
            if isinstance(point, list) and len(point) > index:
                xs.append(float(row["timestamp_ns"]) / 1e9); ys.append(float(point[index]))
        axes[0, 1].plot(xs, ys, label=label)
    style_axis(axes[0, 1], "Aimer selected future point", "World coordinate (m)"); axes[0, 1].legend()
    for key, label in [("solver_tracker", "Solver+Tracker"), ("aimer", "Aimer"), ("shooter", "Shooter"), ("total", "Total")]:
        x, y = nested_series(outputs, ("timing_ms", key)); axes[1, 0].plot(x, y, label=label, linewidth=1)
    style_axis(axes[1, 0], "Per-frame measured processing latency", "Wall-clock duration (ms)"); axes[1, 0].legend(fontsize=8)
    states = {name: i for i, name in enumerate(("lost", "detecting", "tracking", "temp_lost"))}
    x = [float(row["timestamp_ns"]) / 1e9 for row in outputs]
    y = [states.get(row.get("tracker", {}).get("state"), -1) for row in outputs]
    axes[1, 1].step(x, y, where="post"); style_axis(axes[1, 1], "Tracker finite-state machine", "State code")
    axes[1, 1].set_yticks(list(states.values()), list(states.keys()))
    save(fig, out, "04_commands_and_runtime.png")

    # 05: firing outcome and miss distance.
    fig, axes = plt.subplots(1, 2, figsize=(14, 4.8))
    valid_shots = [shot for shot in shots if shot.get("physical_evaluation_valid")]
    sx = [float(shot["command_timestamp_ns"]) / 1e9 for shot in valid_shots]
    sy = [float(shot["miss_distance"]) for shot in valid_shots if number(shot.get("miss_distance")) is not None]
    colors = ["#16a34a" if shot.get("hit") else "#dc2626" for shot in valid_shots]
    axes[0].scatter(sx[:len(sy)], sy, c=colors[:len(sy)], s=13)
    style_axis(axes[0], "Physical shot result (green=hit, red=miss)", "Closest distance to intended armor (m)")
    hit_count = sum(bool(shot.get("hit")) for shot in valid_shots)
    axes[1].bar(["Hit", "Miss", "Unevaluable"], [hit_count, len(valid_shots)-hit_count, len(shots)-len(valid_shots)], color=["#16a34a", "#dc2626", "#9ca3af"])
    axes[1].set_title("Shot outcome counts"); axes[1].set_xlabel("Physical evaluation category"); axes[1].set_ylabel("Number of fired shots"); axes[1].grid(True, axis="y", alpha=.3)
    save(fig, out, "05_shots_and_hits.png")

    readme = f"""# Visualization report

Source run: `{run_dir}`

All time-series plots use **elapsed dataset time in seconds** on the x-axis (timestamp_ns / 1e9). Blank early sections normally mean warm-up frames were excluded or a metric was unavailable.

## Images

- `01_solver_tracker_errors.png`: Solver observation errors and Tracker center/rotation errors. Lower is better. Position is metres, velocity is m/s, angles are radians.
- `02_aimer_and_association.png`: Aimer 3D point error, command angular errors, ballistic pitch diagnostic, and internal-ID to physical-GT-ID association. Different ID curves are not automatically an error because their numbering spaces can have different phases.
- `03_tracker_ekf_states.png`: Raw EKF state estimates over time. These plots show estimated motion/model parameters rather than errors.
- `04_commands_and_runtime.png`: Command angles, future aim-point world coordinates, measured module runtime, and Tracker state transitions.
- `05_shots_and_hits.png`: Per-shot physical miss distance and total hit/miss counts. Green points hit; red points miss.

## Key summary values

- Backend: `{summary.get('backend')}`
- Frames: `{metric(summary, 'frames', 'total')}`; tracking ratio: `{metric(summary, 'frames', 'tracking_ratio')}`
- Solver position RMSE: `{metric(summary, 'solver', 'position', 'rmse')}` m
- Tracker position RMSE: `{metric(summary, 'tracker', 'position', 'rmse')}` m
- Aimer point RMSE: `{metric(summary, 'aimer', 'aim_point', 'rmse')}` m
- Hit rate: `{metric(summary, 'hit', 'hit_rate')}`
- Total latency p95: `{metric(summary, 'latency_ms', 'p95_abs')}` ms
"""
    (out / "README.md").write_text(readme, encoding="utf-8")
    print(f"Visualized run: {run_dir}")
    print(f"Report written to: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
