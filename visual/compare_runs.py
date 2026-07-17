from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


def nested(data: dict, *keys: str):
    value = data
    for key in keys:
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    return value


def main() -> int:
    parser = argparse.ArgumentParser(description="Compare offline evaluation runs")
    parser.add_argument("output_root", nargs="?", default=str(Path(__file__).resolve().parents[1] / "output"))
    parser.add_argument("--csv", default="run_comparison.csv")
    args = parser.parse_args()
    root = Path(args.output_root)
    rows = []
    for summary_path in sorted(root.glob("run_*/summary.json")):
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        rows.append(
            {
                "run": summary_path.parent.name,
                "backend": summary.get("backend"),
                "tracking_ratio": nested(summary, "frames", "tracking_ratio"),
                "tracker_position_rmse": nested(summary, "tracker", "position", "rmse"),
                "tracker_yaw_rmse": nested(summary, "tracker", "yaw", "rmse"),
                "aim_point_rmse": nested(summary, "aimer", "aim_point", "rmse"),
                "shots": nested(summary, "hit", "shots"),
                "hit_rate": nested(summary, "hit", "hit_rate"),
                "latency_p95_ms": nested(summary, "latency_ms", "p95_abs"),
            }
        )
    target = root / args.csv
    fields = ["run", "backend", "tracking_ratio", "tracker_position_rmse", "tracker_yaw_rmse", "aim_point_rmse", "shots", "hit_rate", "latency_p95_ms"]
    with target.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    print(target)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

