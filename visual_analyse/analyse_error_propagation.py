"""Compute and plot six distance metrics along the offline auto-aim pipeline.

The alignment key is frame_id: all metrics on one x position originate from the
same Solver input frame.  Aimer and ballistic truth are still sampled at their
own future/impact reference timestamps, rather than incorrectly at frame time.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any

OFFLINE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(OFFLINE_ROOT))

from utils.hit_checker import simulate_physical_shot  # noqa: E402
from utils.io_utils import load_config, read_jsonl  # noqa: E402
from utils.math_utils import distance3  # noqa: E402
from utils.timeline import StateTimeline  # noqa: E402

try:
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MaxNLocator
except ImportError as exc:
    print("Install matplotlib with: python3 -m pip install -r offline_test/visual_analyse/requirements.txt", file=sys.stderr)
    raise SystemExit(2) from exc


METRICS = [
    ("solver_armor_error", "Solver armor mean"),
    ("tracker_current_armor_error", "Tracker reconstructed armor mean"),
    ("tracker_center_error", "Tracker center"),
    ("aimer_future_armor_error", "Aimer future armor"),
    ("bullet_center_error", "Virtual bullet to armor center"),
    ("physical_miss_distance", "Virtual physical miss"),
]


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def finite(value: Any) -> float | None:
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (TypeError, ValueError):
        return None


def resolve_run_dir(value: Path) -> Path:
    candidates = [value, OFFLINE_ROOT.parent / value, OFFLINE_ROOT / value]
    parts = value.parts
    if parts and parts[0].lower() == "offline_test":
        candidates.append(OFFLINE_ROOT.joinpath(*parts[1:]))
    for candidate in candidates:
        resolved = candidate.expanduser().resolve()
        if (resolved / "manifest.json").exists():
            return resolved
    checked = "\n  ".join(str(path.expanduser().resolve()) for path in candidates)
    raise FileNotFoundError(f"Cannot find run directory. Checked:\n  {checked}")


def dataset_dir_for(run_dir: Path, manifest: dict[str, Any]) -> Path:
    recorded = Path(str(manifest["dataset"]))
    if recorded.exists():
        return recorded.resolve()
    fallback = OFFLINE_ROOT / "data" / "cases" / recorded.name
    if fallback.exists():
        return fallback.resolve()
    raise FileNotFoundError(f"Dataset is unavailable: recorded={recorded}, fallback={fallback}")


def tracker_armors(row: dict[str, Any], armor_count: int, dt: float = 0.0) -> list[list[float]]:
    x = float(row["estimate_x"]) + float(row["estimate_vx"]) * dt
    y = float(row["estimate_y"]) + float(row["estimate_vy"]) * dt
    z = float(row["estimate_z"]) + float(row["estimate_vz"]) * dt
    yaw = float(row["estimate_yaw"]) + float(row["estimate_yaw_rate"]) * dt
    radius = float(row["estimate_radius"])
    radius_delta = float(row["estimate_radius_delta"])
    height_delta = float(row["estimate_height_delta"])
    result = []
    for armor_id in range(armor_count):
        angle = yaw + armor_id * 2.0 * math.pi / armor_count
        alternate = armor_count == 4 and armor_id in (1, 3)
        r = radius + radius_delta if alternate else radius
        result.append([x - r * math.cos(angle), y - r * math.sin(angle), z + height_delta if alternate else z])
    return result


def mean(values: list[float]) -> float | None:
    return sum(values) / len(values) if values else None


def write_results(path: Path, rows: list[dict[str, Any]]) -> None:
    fields = ["frame_id", "source_timestamp_ns", "aimer_reference_timestamp_ns", *[key for key, _ in METRICS]]
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows({key: row.get(key) for key in fields} for row in rows)


def plot(rows: list[dict[str, Any]], output_dir: Path) -> None:
    def scale_max(metrics: list[tuple[str, str]]) -> float:
        values = [float(row[key]) for row in rows for key, _ in metrics if row.get(key) is not None]
        maximum = max(values, default=1.0)
        return maximum * 1.05 if maximum > 0 else 1.0

    algorithm_max = scale_max(METRICS[:4])
    ballistic_max = scale_max(METRICS[4:])
    all_metrics_max = max(algorithm_max, ballistic_max)

    fig, axes = plt.subplots(3, 2, figsize=(15, 12), sharex=True, sharey=False)
    for metric_index, (ax, (key, label)) in enumerate(zip(axes.flat, METRICS)):
        points = [(int(row["source_timestamp_ns"]) / 1e9, row.get(key)) for row in rows if row.get(key) is not None]
        if points:
            ax.plot([p[0] for p in points], [p[1] for p in points], linewidth=1.05)
        ax.set_title(label)
        ax.set_xlabel("Solver input time (s)")
        ax.set_ylabel("Distance (m)")
        ax.set_ylim(0, algorithm_max if metric_index < 4 else ballistic_max)
        ax.grid(True, alpha=.3)
        ax.xaxis.set_major_locator(MaxNLocator(8))
    fig.suptitle("Six pipeline distance metrics (two shared y-scale groups)", fontsize=15)
    fig.tight_layout()
    fig.savefig(output_dir / "01_six_metrics_over_time.png", dpi=180, bbox_inches="tight")
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(15, 12), sharex=True)
    for key, label in METRICS:
        points = [(int(row["frame_id"]), row.get(key)) for row in rows if row.get(key) is not None]
        if points:
            axes[0].plot([p[0] for p in points], [p[1] for p in points], linewidth=1.0, label=label)
    axes[0].set_title("All six metrics aligned by the same Solver input frame")
    axes[0].set_ylabel("Distance (m)")
    axes[0].set_ylim(0, all_metrics_max)
    axes[0].grid(True, alpha=.3)
    axes[0].legend(fontsize=8, ncol=2)

    for key, label in METRICS[:4]:
        points = [(int(row["frame_id"]), row.get(key)) for row in rows if row.get(key) is not None]
        if points:
            axes[1].plot([p[0] for p in points], [p[1] for p in points], linewidth=1.0, label=label)
    axes[1].set_title("First four algorithm-stage metrics aligned by Solver input frame")
    axes[1].set_xlabel("Solver input frame_id")
    axes[1].set_ylabel("Distance (m)")
    axes[1].set_ylim(0, algorithm_max)
    axes[1].grid(True, alpha=.3)
    axes[1].legend(fontsize=8, ncol=2)
    fig.tight_layout()
    fig.savefig(output_dir / "02_six_metrics_aligned_by_solver_input.png", dpi=180, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyse six error-propagation distance metrics")
    parser.add_argument("run_dir", type=Path, help="offline_test/output/run_* directory")
    parser.add_argument("--output-dir", type=Path, help="default: RUN_DIR/error_propagation_report")
    args = parser.parse_args()
    run_dir = resolve_run_dir(args.run_dir)

    required = ["manifest.json", "solver_comparison.csv", "tracker_comparison.csv", "aimer_comparison.csv", "command_comparison.csv", "algorithm_output.jsonl", "run_config.yaml"]
    missing = [name for name in required if not (run_dir / name).exists()]
    if missing:
        raise FileNotFoundError(f"Run is missing required evaluation files: {missing}")

    manifest = json.loads((run_dir / "manifest.json").read_text(encoding="utf-8"))
    config = load_config(run_dir / "run_config.yaml")
    dataset_dir = dataset_dir_for(run_dir, manifest)
    solver_rows = read_csv(run_dir / "solver_comparison.csv")
    tracker_rows = read_csv(run_dir / "tracker_comparison.csv")
    aimer_rows = read_csv(run_dir / "aimer_comparison.csv")
    command_rows = read_csv(run_dir / "command_comparison.csv")
    outputs = read_jsonl(run_dir / "algorithm_output.jsonl")
    frames = read_jsonl(dataset_dir / "frames.jsonl")
    target_truth = read_jsonl(dataset_dir / "ground_truth" / "target_states.jsonl")
    armor_truth = read_jsonl(dataset_dir / "ground_truth" / "armor_states.jsonl")
    armor_timeline = StateTimeline(armor_truth, ("target_id", "armor_id"))

    solver_by_frame: dict[int, list[dict[str, str]]] = defaultdict(list)
    for row in solver_rows:
        solver_by_frame[int(row["frame_id"])].append(row)
    tracker_by_frame = {int(row["frame_id"]): row for row in tracker_rows}
    aimer_by_frame = {int(row["frame_id"]): row for row in aimer_rows}
    command_by_frame = {int(row["frame_id"]): row for row in command_rows}
    output_by_frame = {int(row["frame_id"]): row for row in outputs}
    frame_by_id = {int(row["frame_id"]): row for row in frames}
    armor_count = int(target_truth[0].get("armor_count", 4)) if target_truth else 4
    physics = config["simulation"]["ballistics"]
    geometry = config["armor_geometry"]

    frame_ids = sorted(set(solver_by_frame) | set(tracker_by_frame) | set(aimer_by_frame))
    result_rows: list[dict[str, Any]] = []
    for frame_id in frame_ids:
        solver_frame = solver_by_frame.get(frame_id, [])
        tracker = tracker_by_frame.get(frame_id)
        aimer = aimer_by_frame.get(frame_id)
        command = command_by_frame.get(frame_id)
        output = output_by_frame.get(frame_id, {})
        source_timestamp = int(
            (solver_frame[0]["source_timestamp_ns"] if solver_frame else
             tracker["source_timestamp_ns"] if tracker else aimer["source_timestamp_ns"])
        )
        row: dict[str, Any] = {
            "frame_id": frame_id,
            "source_timestamp_ns": source_timestamp,
            "aimer_reference_timestamp_ns": int(aimer["reference_timestamp_ns"]) if aimer else None,
        }

        solver_errors = [float(item["position_error"]) for item in solver_frame if finite(item.get("position_error")) is not None]
        row["solver_armor_error"] = mean(solver_errors)

        if tracker is not None:
            row["tracker_center_error"] = finite(tracker.get("position_error"))
            reconstructed = tracker_armors(tracker, armor_count)
            reconstructed_errors = []
            for solver_item in solver_frame:
                truth_position = [float(solver_item[f"truth_{axis}"]) for axis in "xyz"]
                reconstructed_errors.append(min(distance3(point, truth_position) for point in reconstructed))
            row["tracker_current_armor_error"] = mean(reconstructed_errors)

        if aimer is not None:
            row["aimer_future_armor_error"] = finite(aimer.get("position_error"))

        if aimer is not None and command is not None and output and frame_id in frame_by_id:
            target_id = int(aimer["target_id"])
            armor_id = int(aimer["armor_id"])
            aimer_output = output.get("aimer", {})
            delay_s = finite(aimer_output.get("delay_time_s")) or 0.0
            muzzle_timestamp = source_timestamp + int(round(delay_s * 1e9))
            virtual = simulate_physical_shot(
                float(command["estimate_yaw"]), float(command["estimate_pitch"]),
                float(frame_by_id[frame_id]["bullet_speed"]), muzzle_timestamp,
                target_id, armor_id, armor_timeline, geometry, physics,
            )
            if virtual["evaluation_valid"]:
                impact_truth = armor_timeline.sample(int(virtual["impact_timestamp_ns"]), target_id, armor_id)
                if impact_truth is not None and virtual["impact_position"] is not None:
                    row["bullet_center_error"] = distance3(virtual["impact_position"], impact_truth["position"])
                row["physical_miss_distance"] = finite(virtual.get("miss_distance"))

        result_rows.append(row)

    output_dir = (args.output_dir or (run_dir / "error_propagation_report")).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    write_results(output_dir / "error_propagation.csv", result_rows)
    plot(result_rows, output_dir)
    valid_counts = {key: sum(row.get(key) is not None for row in result_rows) for key, _ in METRICS}
    (output_dir / "README.md").write_text(
        "# Error propagation report\n\n"
        "All six values are Euclidean/physical distances in metres and are aligned by the originating `frame_id`. "
        "Aimer uses its future reference timestamp; virtual ballistic metrics use the simulated plane-crossing timestamp.\n\n"
        "`physical_miss_distance` is zero inside the armor rectangle; `bullet_center_error` remains nonzero unless the bullet crosses its center.\n\n"
        f"Valid samples: `{json.dumps(valid_counts, ensure_ascii=False)}`\n",
        encoding="utf-8",
    )
    print(f"Analysed run: {run_dir}")
    print(f"Report written to: {output_dir}")
    print(f"Valid samples: {valid_counts}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
