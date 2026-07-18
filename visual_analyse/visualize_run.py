"""Create a module-oriented statistical and visual report for one run."""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Any

try:
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MaxNLocator
except ImportError as exc:
    print("Install matplotlib with: python -m pip install -r offline_test/visual/requirements.txt", file=sys.stderr)
    raise SystemExit(2) from exc


def read_csv(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def num(value: Any) -> float | None:
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (TypeError, ValueError):
        return None


def xy(rows: list[dict[str, Any]], key: str) -> tuple[list[float], list[float]]:
    points = []
    for row in rows:
        timestamp = num(row.get("source_timestamp_ns", row.get("timestamp_ns")))
        value = num(row.get(key))
        if timestamp is not None and value is not None:
            points.append((timestamp / 1e9, value))
    return [p[0] for p in points], [p[1] for p in points]


def style(ax: Any, title: str, ylabel: str) -> None:
    ax.set_title(title)
    ax.set_xlabel("Algorithm output time (s)")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=.3)
    ax.xaxis.set_major_locator(MaxNLocator(8))


def lines(ax: Any, rows: list[dict[str, Any]], specs: list[tuple[str, str]], title: str, ylabel: str) -> None:
    for key, label in specs:
        x, y = xy(rows, key)
        if not y:
            continue

        is_truth = "truth" in label.lower() or "gt" in label.lower()

        ax.plot(
            x,
            y,
            linewidth=1.15,
            label=label,
            alpha=0.7,
            zorder=1 if is_truth else 2,
        )
    style(ax, title, ylabel)
    if ax.lines:
        ax.legend(fontsize=8)


def save(fig: Any, out: Path, name: str) -> None:
    fig.tight_layout()
    fig.savefig(out / name, dpi=180, bbox_inches="tight")
    plt.close(fig)


def position_figure(rows: list[dict[str, Any]], title: str, out: Path, name: str, truth_label: str) -> None:
    fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
    for index, axis in enumerate("xyz"):
        lines(axes[index, 0], rows, [(f"estimate_{axis}", "Estimate"), (f"truth_{axis}", truth_label)], f"{title}: {axis} estimate vs truth", f"{axis} (m)")
        lines(axes[index, 1], rows, [(f"error_{axis}", "Signed error")], f"{title}: {axis} signed error", "Error (m)")
        axes[index, 1].axhline(0, color="black", linewidth=.7)
    save(fig, out, name)


def grouped_armor_rows(rows: list[dict[str, Any]]) -> list[tuple[tuple[int, int], list[list[dict[str, Any]]]]]:
    """Group Solver rows by physical armor and split gaps to prevent false connections."""
    groups: dict[tuple[int, int], list[dict[str, Any]]] = {}
    for row in rows:
        target_id = num(row.get("target_id"))
        armor_id = num(row.get("armor_id"))
        frame_id = num(row.get("frame_id"))
        if target_id is None or armor_id is None or frame_id is None:
            continue
        groups.setdefault((int(target_id), int(armor_id)), []).append(row)

    result = []
    for key, group in sorted(groups.items()):
        ordered = sorted(group, key=lambda row: int(float(row["frame_id"])))
        segments: list[list[dict[str, Any]]] = []
        for row in ordered:
            if not segments or int(float(row["frame_id"])) > int(float(segments[-1][-1]["frame_id"])) + 1:
                segments.append([])
            segments[-1].append(row)
        result.append((key, segments))
    return result


def plot_grouped_armor_value(
    ax: Any, grouped: list[tuple[tuple[int, int], list[list[dict[str, Any]]]]],
    estimate_key: str, truth_key: str | None, title: str, ylabel: str,
) -> None:
    colors = plt.get_cmap("tab10")
    for color_index, ((target_id, armor_id), segments) in enumerate(grouped):
        color = colors(color_index % 10)
        for segment_index, segment in enumerate(segments):
            x, estimate = xy(segment, estimate_key)
            if estimate:
                ax.plot(x, estimate, color=color, linewidth=1.2,
                        label=f"T{target_id} A{armor_id} estimate" if segment_index == 0 else None)
            if truth_key is not None:
                tx, truth = xy(segment, truth_key)
                if truth:
                    ax.plot(tx, truth, color=color, linestyle="--", linewidth=1.0, alpha=.8,
                            label=f"T{target_id} A{armor_id} truth" if segment_index == 0 else None)
    style(ax, title, ylabel)
    if ax.lines:
        ax.legend(fontsize=7, ncol=2)


def solver_position_figure(rows: list[dict[str, Any]], out: Path) -> None:
    grouped = grouped_armor_rows(rows)
    fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
    for index, axis in enumerate("xyz"):
        plot_grouped_armor_value(
            axes[index, 0], grouped, f"estimate_{axis}", f"truth_{axis}",
            f"Solver current armors: {axis} estimate vs truth", f"{axis} (m)",
        )
        plot_grouped_armor_value(
            axes[index, 1], grouped, f"error_{axis}", None,
            f"Solver current armors: {axis} signed error", "Error (m)",
        )
        axes[index, 1].axhline(0, color="black", linewidth=.7)
    save(fig, out, "02_solver_position.png")


def solver_yaw_figure(rows: list[dict[str, Any]], out: Path) -> None:
    grouped = grouped_armor_rows(rows)
    fig, axes = plt.subplots(1, 2, figsize=(14, 4.8), sharex=True)
    plot_grouped_armor_value(
        axes[0], grouped, "estimate_yaw", "truth_yaw",
        "Solver current armors: yaw estimate vs truth", "Yaw (rad)",
    )
    plot_grouped_armor_value(
        axes[1], grouped, "error_yaw", None,
        "Solver current armors: yaw signed error", "Error (rad)",
    )
    axes[1].axhline(0, color="black", linewidth=.7)
    save(fig, out, "03_solver_yaw.png")


def paired_figure(rows: list[dict[str, Any]], items: list[tuple[str, str, str]], title: str, out: Path, name: str) -> None:
    fig, axes = plt.subplots(len(items), 2, figsize=(14, 3.2 * len(items)), squeeze=False, sharex=True)
    for index, (key, label, unit) in enumerate(items):
        lines(axes[index, 0], rows, [(f"estimate_{key}", "Estimate"), (f"truth_{key}", "Ground Truth")], f"{title}: {label}", unit)
        lines(axes[index, 1], rows, [(f"error_{key}", "Signed error")], f"{title}: {label} error", unit)
        axes[index, 1].axhline(0, color="black", linewidth=.7)
    save(fig, out, name)


def main() -> int:
    parser = argparse.ArgumentParser(description="Visualize one completed offline-test run")
    parser.add_argument("run_dir", type=Path, help="explicit offline_test/output/run_* directory")
    args = parser.parse_args()
    run_dir = args.run_dir.resolve()
    required = ["summary.json", "tracker_comparison.csv", "aimer_comparison.csv", "command_comparison.csv"]
    missing = [name for name in required if not (run_dir / name).exists()]
    if missing:
        raise FileNotFoundError(f"Run has not been evaluated with the comparison schema; missing {missing}")

    out = run_dir / "visual_report"
    out.mkdir(exist_ok=True)
    summary = read_json(run_dir / "summary.json")
    observation = read_csv(run_dir / "observation_comparison.csv")
    solver = read_csv(run_dir / "solver_comparison.csv")
    tracker = read_csv(run_dir / "tracker_comparison.csv")
    aimer = read_csv(run_dir / "aimer_comparison.csv")
    command = read_csv(run_dir / "command_comparison.csv")
    outputs = read_jsonl(run_dir / "algorithm_output.jsonl")
    shots = read_jsonl(run_dir / "shots.jsonl")

    fig, axes = plt.subplots(2, 2, figsize=(14, 8), sharex=True)
    lines(axes[0, 0], observation, [("corner_rmse_px", "Corner RMSE")], "Observation corner error", "RMSE (px)")
    lines(axes[0, 1], observation, [("center_error_u", "u"), ("center_error_v", "v")], "Observation center signed error", "Error (px)")
    lines(axes[1, 0], observation, [("width_error_px", "Width"), ("height_error_px", "Height")], "Observation size signed error", "Error (px)")
    detected = [1.0 if row.get("detected", "").lower() == "true" else 0.0 for row in observation]
    tx, _ = xy(observation, "frame_id")
    axes[1, 1].step(tx[:len(detected)], detected, where="post")
    style(axes[1, 1], "Observation detection state", "Detected (0/1)")
    save(fig, out, "01_observation.png")

    solver_position_figure(solver, out)
    solver_yaw_figure(solver, out)
    position_figure(tracker, "Tracker current target center", out, "04_tracker_position.png", "Current center GT")
    paired_figure(tracker, [("vx", "vx", "m/s"), ("vy", "vy", "m/s"), ("vz", "vz", "m/s")], "Tracker velocity", out, "05_tracker_velocity.png")

    fig, axes = plt.subplots(5, 1, figsize=(14, 14), sharex=True)
    for ax, (key, label, unit) in zip(axes, [("yaw", "Yaw", "rad"), ("yaw_rate", "Yaw rate", "rad/s"), ("radius", "Radius", "m"), ("radius_delta", "Radius delta", "m"), ("height_delta", "Height delta", "m")]):
        lines(ax, tracker, [(f"estimate_{key}", "Estimate"), (f"truth_{key}", "Ground Truth")], f"Tracker model: {label}", unit)
    save(fig, out, "06_tracker_rotation_model.png")

    position_figure(aimer, "Aimer future armor at impact time", out, "07_aimer_future_position.png", "Future armor GT")
    paired_figure(command, [("yaw", "command yaw", "rad"), ("pitch", "ballistic command pitch", "rad")], "Command vs physical ideal", out, "08_command.png")

    fig, axes = plt.subplots(1, 2, figsize=(14, 4.8))
    valid_shots = [shot for shot in shots if shot.get("physical_evaluation_valid") and num(shot.get("miss_distance")) is not None]
    sx = [float(shot["command_timestamp_ns"]) / 1e9 for shot in valid_shots]
    sy = [float(shot["miss_distance"]) for shot in valid_shots]
    colors = ["#16a34a" if shot.get("hit") else "#dc2626" for shot in valid_shots]
    axes[0].scatter(sx, sy, c=colors, s=14)
    style(axes[0], "Physical shots (green=hit, red=miss)", "Miss distance (m)")
    hits = sum(bool(shot.get("hit")) for shot in valid_shots)
    axes[1].bar(["Hit", "Miss", "Unevaluable"], [hits, len(valid_shots)-hits, len(shots)-len(valid_shots)], color=["#16a34a", "#dc2626", "#9ca3af"])
    axes[1].set_title("Shot outcome counts"); axes[1].set_ylabel("Shots"); axes[1].grid(axis="y", alpha=.3)
    save(fig, out, "09_physical_hits.png")

    fig, axes = plt.subplots(2, 2, figsize=(14, 8), sharex=True)
    timing_rows = [{"source_timestamp_ns": row["timestamp_ns"], **row.get("timing_ms", {})} for row in outputs]
    lines(axes[0, 0], timing_rows, [("solver_tracker", "Solver+Tracker"), ("aimer", "Aimer"), ("shooter", "Shooter"), ("total", "Total")], "Module runtime", "ms")
    lines(axes[0, 1], tracker, [("tracker_state", "State")], "Tracker state (see CSV for labels)", "State")
    lines(axes[1, 0], aimer, [("aimer_internal_armor_id", "Internal ID"), ("armor_id", "Matched physical ID")], "Armor association audit", "Armor ID")
    lines(axes[1, 1], command, [("line_of_sight_pitch", "LOS pitch"), ("truth_pitch", "Ballistic ideal")], "Pitch reference diagnostic", "rad")
    save(fig, out, "10_debug_appendix.png")

    components = lambda module: summary.get(module, {}).get("position_components", {})
    report = [
        "# Module-oriented visualization report", "", f"Source run: `{run_dir}`", "",
        "The x-axis is the current algorithm output time. Aimer/Command Ground Truth is sampled at `reference_timestamp_ns`, the predicted impact time.", "",
        "## Position error propagation", "",
        "| Layer | x bias | y bias | z bias | position RMSE | position P95 |", "|---|---:|---:|---:|---:|---:|",
    ]
    for module in ("solver", "tracker", "aimer"):
        values = components(module)
        report.append(f"| {module.title()} | {values.get('x_bias')} | {values.get('y_bias')} | {values.get('z_bias')} | {values.get('position_rmse')} | {values.get('position_p95')} |")
    report += ["", "## Report split", "", "Images 01–09 are the core performance report. `10_debug_appendix.png` contains runtime, association, state and pitch-reference diagnostics."]
    (out / "README.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    print(f"Report written to: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
