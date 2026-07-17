from __future__ import annotations

import argparse
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

from utils.evaluator import evaluate
from utils.generator import generate_dataset
from utils.io_utils import dump_json, load_config, make_run_dir, prepare_algorithm_input, sha256_file
from utils.mock_runner import run_mock
from utils.report import print_summary, write_svg_plot
from utils.validation import validate_dataset

PROJECT_ROOT = Path(__file__).resolve().parent


def _run_cpp(algorithm_input_dir: Path, run_dir: Path, config: dict, root: Path) -> None:
    executable = (root / config["runner"]["cpp_executable"]).resolve()
    if not executable.exists() and sys.platform == "win32" and executable.suffix.lower() != ".exe":
        executable = executable.with_suffix(".exe")
    auto_config = (root / config["tested_program"]["auto_aim_config"]).resolve()
    if not executable.exists():
        raise FileNotFoundError(
            f"C++ runner not found: {executable}. Build it first or set runner.backend to 'mock'."
        )
    command = [str(executable), "--dataset", str(algorithm_input_dir), "--output", str(run_dir), "--auto-config", str(auto_config)]
    completed = subprocess.run(command, cwd=root, check=False)
    if completed.returncode != 0:
        raise RuntimeError(f"C++ runner failed with exit code {completed.returncode}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Offline auto-aim evaluation pipeline")
    parser.add_argument("--config", default="config/offline_test.yaml", help="config path relative to offline_test")
    parser.add_argument("--backend", choices=("mock", "cpp"), help="override runner.backend")
    parser.add_argument("--case", choices=("static_target", "linear_target", "spinning_target"), help="override the configured generated case")
    parser.add_argument("--dataset-only", action="store_true", help="only generate/locate the dataset")
    args = parser.parse_args()

    config_path = (PROJECT_ROOT / args.config).resolve()
    config = load_config(config_path)
    if args.backend:
        config["runner"]["backend"] = args.backend
    if args.case:
        config["data"]["case_config"] = f"config/cases/{args.case}.yaml"
    dataset_dir = generate_dataset(config, PROJECT_ROOT) if config["data"]["mode"] == "generated" else (PROJECT_ROOT / config["data"]["dataset_path"]).resolve()
    dataset_counts = validate_dataset(dataset_dir)
    if args.dataset_only:
        print(dataset_dir)
        return 0

    run_dir = make_run_dir((PROJECT_ROOT / config["output"]["root"]).resolve(), Path(config["data"]["case_config"]).stem)
    effective_config_path = run_dir / "run_config.yaml"
    dump_json(effective_config_path, config)
    manifest = {
        "schema_version": "1.0",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "source_config": str(config_path),
        "source_config_sha256": sha256_file(config_path),
        "effective_config": str(effective_config_path),
        "effective_config_sha256": sha256_file(effective_config_path),
        "command_line_overrides": {
            "backend": args.backend,
            "case": args.case,
        },
        "dataset": str(dataset_dir),
        "backend": config["runner"]["backend"],
        "algorithm_input_policy": {
            "allowed_files": ["metadata.yaml", "frames.jsonl", "observations.jsonl"],
            "ground_truth_available": False,
        },
        "auto_aim_config": config["tested_program"]["auto_aim_config"],
        "coordinate_system": "world: x-forward, y-left, z-up",
        "angle_unit": "rad",
        "position_unit": "m",
        "timestamp_unit": "ns",
        "hit_model": config["simulation"]["ballistic_model"],
        "dataset_counts": dataset_counts,
    }
    dump_json(run_dir / "manifest.json", manifest)

    algorithm_input_dir = prepare_algorithm_input(dataset_dir, run_dir / "algorithm_input")
    if config["runner"]["backend"] == "mock":
        run_mock(algorithm_input_dir, run_dir, config)
    else:
        _run_cpp(algorithm_input_dir, run_dir, config, PROJECT_ROOT)
    summary = evaluate(dataset_dir, run_dir, config)
    if config["output"].get("write_svg", True):
        write_svg_plot(run_dir)
    print_summary(summary, run_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
