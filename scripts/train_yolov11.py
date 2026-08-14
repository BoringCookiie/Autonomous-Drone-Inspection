#!/usr/bin/env python3
"""Train and validate a reproducible YOLO11 earthen-defect baseline."""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-yaml", type=Path, default=ROOT / "data/earthen_defects.yaml")
    parser.add_argument("--model", default="yolo11s.pt", help="Ultralytics YOLO11 checkpoint")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--batch", "--batch-size", dest="batch", type=int, default=16)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default="", help="Examples: '', 'cpu', '0', '0,1'")
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--patience", type=int, default=25)
    parser.add_argument("--name", default="earthen_yolo11")
    parser.add_argument("--output", type=Path, default=ROOT / "models/yolo/yolo_earthen_v11.pt")
    parser.add_argument("--results-dir", type=Path, default=ROOT / "results/yolo")
    parser.add_argument("--config", type=Path, default=ROOT / "data/augmentation.yaml")
    parser.add_argument("--resume", action="store_true")
    return parser.parse_args()


def load_yaml(path: Path) -> dict:
    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError("PyYAML is required: pip install -r requirements-yolo.txt") from exc
    with path.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def ensure_dataset_ready(data_yaml: Path) -> None:
    if not data_yaml.is_file():
        raise FileNotFoundError(f"Dataset YAML not found: {data_yaml}")
    cfg = load_yaml(data_yaml)
    base = Path(cfg.get("path", data_yaml.parent))
    if not base.is_absolute():
        base = (data_yaml.parent / base).resolve()
    missing = []
    for split in ("train", "val", "test"):
        value = cfg.get(split)
        target = base / value if value else None
        if target is None or not target.is_dir() or not any(target.rglob("*.*")):
            missing.append(f"{split}: {target}")
    if missing:
        raise RuntimeError("Dataset is empty/incomplete; run scripts/prepare_yolo_dataset.py. Missing: " + ", ".join(missing))


def metric_value(metrics: object, key: str) -> float | None:
    value = getattr(metrics, "results_dict", {}).get(key)
    return float(value) if value is not None else None


def main() -> int:
    args = parse_args()
    try:
        from ultralytics import YOLO
    except ImportError:
        print("ERROR: ultralytics is missing. Install requirements-yolo.txt", file=sys.stderr)
        return 2
    try:
        ensure_dataset_ready(args.data_yaml.resolve())
        augmentation = load_yaml(args.config.resolve())
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    args.results_dir.mkdir(parents=True, exist_ok=True)
    model = YOLO(args.model)
    model.train(
        data=str(args.data_yaml.resolve()), epochs=args.epochs, batch=args.batch,
        imgsz=args.imgsz, device=args.device or None, workers=args.workers,
        seed=args.seed, deterministic=True, patience=args.patience,
        project=str(args.results_dir / "runs"), name=args.name, exist_ok=True,
        resume=args.resume, val=True, plots=True, **augmentation,
    )
    save_dir = Path(model.trainer.save_dir)
    best = save_dir / "weights/best.pt"
    if not best.is_file():
        print(f"ERROR: training completed without best weights at {best}", file=sys.stderr)
        return 1
    args.output.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(best, args.output)

    metrics = YOLO(str(args.output)).val(
        data=str(args.data_yaml.resolve()), split="test", imgsz=args.imgsz,
        batch=args.batch, device=args.device or None, project=str(args.results_dir),
        name="test", exist_ok=True, plots=True,
    )
    precision = metric_value(metrics, "metrics/precision(B)")
    recall = metric_value(metrics, "metrics/recall(B)")
    f1 = 2 * precision * recall / (precision + recall) if precision is not None and recall is not None and precision + recall else None
    speed = getattr(metrics, "speed", {})
    inference_ms = float(speed.get("inference")) if speed.get("inference") is not None else None
    report = {
        "status": "COMPLETED", "weights": str(args.output.resolve()),
        "data_yaml": str(args.data_yaml.resolve()), "seed": args.seed,
        "metrics": {"precision": precision, "recall": recall, "f1": f1,
                    "map50": metric_value(metrics, "metrics/mAP50(B)"),
                    "map50_95": metric_value(metrics, "metrics/mAP50-95(B)"),
                    "inference_ms_per_image": inference_ms,
                    "fps": 1000.0 / inference_ms if inference_ms and inference_ms > 0 else None},
        "run_directory": str(save_dir.resolve()),
    }
    (args.results_dir / "metrics.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
