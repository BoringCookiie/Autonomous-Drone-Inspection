#!/usr/bin/env python3
"""Validate, deduplicate and split existing YOLO-labelled source datasets."""

from __future__ import annotations

import argparse
import hashlib
import json
import random
import shutil
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff", ".webp"}
CLASS_NAMES = ("structural_crack", "surface_erosion", "moisture_stain")


@dataclass(frozen=True)
class Sample:
    source: str
    image: Path
    label: Path
    width: int
    height: int
    sha256: str
    perceptual_hash: int
    classes: tuple[int, ...]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", action="append", type=Path, help="YOLO source root; repeatable")
    parser.add_argument("--output", type=Path, default=ROOT / "data/yolo")
    parser.add_argument("--train", type=float, default=0.70)
    parser.add_argument("--val", type=float, default=0.15)
    parser.add_argument("--test", type=float, default=0.15)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--min-width", type=int, default=32)
    parser.add_argument("--min-height", type=int, default=32)
    parser.add_argument("--duplicate-distance", type=int, default=4, help="Maximum 64-bit dHash distance treated as near-duplicate")
    parser.add_argument("--allow-empty", action="store_true", help="Keep verified negative images with empty labels")
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def image_size_and_hash(path: Path) -> tuple[int, int, str, int]:
    try:
        from PIL import Image
    except ImportError as exc:
        raise RuntimeError("Pillow is required: pip install -r requirements-yolo.txt") from exc
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    with Image.open(path) as image:
        image.verify()
    with Image.open(path) as image:
        gray = image.convert("L").resize((9, 8))
        pixels = list(gray.getdata())
        bits = [pixels[row * 9 + col] > pixels[row * 9 + col + 1] for row in range(8) for col in range(8)]
        perceptual_hash = sum(int(bit) << index for index, bit in enumerate(bits))
        return image.width, image.height, digest.hexdigest(), perceptual_hash


def validate_label(path: Path) -> tuple[tuple[int, ...], list[str]]:
    errors: list[str] = []
    classes: list[int] = []
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeError) as exc:
        return (), [f"unreadable label: {exc}"]
    for line_no, line in enumerate(lines, 1):
        if not line.strip():
            continue
        fields = line.split()
        if len(fields) != 5:
            errors.append(f"line {line_no}: expected 5 fields")
            continue
        try:
            class_id = int(fields[0])
            x, y, width, height = map(float, fields[1:])
        except ValueError:
            errors.append(f"line {line_no}: non-numeric value")
            continue
        if class_id not in range(len(CLASS_NAMES)):
            errors.append(f"line {line_no}: invalid class_id {class_id}")
        if not all(0.0 <= value <= 1.0 for value in (x, y, width, height)):
            errors.append(f"line {line_no}: coordinates must be in [0,1]")
        if width <= 0.0 or height <= 0.0:
            errors.append(f"line {line_no}: width and height must be positive")
        if x - width / 2 < 0 or x + width / 2 > 1 or y - height / 2 < 0 or y + height / 2 > 1:
            errors.append(f"line {line_no}: box crosses image boundary")
        classes.append(class_id)
    return tuple(classes), errors


def matching_label(source: Path, image: Path) -> Path | None:
    candidates = [image.with_suffix(".txt")]
    parts = list(image.relative_to(source).parts)
    if "images" in parts:
        parts[parts.index("images")] = "labels"
        candidates.append((source / Path(*parts)).with_suffix(".txt"))
    return next((candidate for candidate in candidates if candidate.is_file()), None)


def inspect_sources(sources: list[Path], args: argparse.Namespace) -> tuple[list[Sample], list[dict]]:
    samples: list[Sample] = []
    issues: list[dict] = []
    seen_hashes: dict[str, Path] = {}
    seen_perceptual: list[tuple[int, Path]] = []
    for source in sources:
        if not source.is_dir():
            issues.append({"source": str(source), "error": "source directory missing"})
            continue
        for image in sorted(path for path in source.rglob("*") if path.suffix.lower() in IMAGE_EXTENSIONS):
            label = matching_label(source, image)
            if label is None:
                issues.append({"image": str(image), "error": "annotation missing"})
                continue
            try:
                width, height, sha256, perceptual_hash = image_size_and_hash(image)
            except Exception as exc:
                issues.append({"image": str(image), "error": f"invalid image: {exc}"})
                continue
            if width < args.min_width or height < args.min_height:
                issues.append({"image": str(image), "error": f"image too small: {width}x{height}"})
                continue
            classes, label_errors = validate_label(label)
            if label_errors:
                issues.append({"image": str(image), "label": str(label), "errors": label_errors})
                continue
            if not classes and not args.allow_empty:
                issues.append({"image": str(image), "error": "empty annotation (use --allow-empty for verified negatives)"})
                continue
            if sha256 in seen_hashes:
                issues.append({"image": str(image), "error": "exact duplicate", "duplicate_of": str(seen_hashes[sha256])})
                continue
            near_duplicate = next((other for other_hash, other in seen_perceptual if (perceptual_hash ^ other_hash).bit_count() <= args.duplicate_distance), None)
            if near_duplicate is not None:
                issues.append({"image": str(image), "error": "perceptual near-duplicate", "duplicate_of": str(near_duplicate)})
                continue
            seen_hashes[sha256] = image
            seen_perceptual.append((perceptual_hash, image))
            samples.append(Sample(source.name, image, label, width, height, sha256, perceptual_hash, classes))
    return samples, issues


def split_samples(samples: list[Sample], ratios: dict[str, float], seed: int) -> dict[str, list[Sample]]:
    """Deterministically split duplicate-free samples, stratified by label signature."""
    rng = random.Random(seed)
    groups: dict[tuple[int, ...], list[Sample]] = defaultdict(list)
    for sample in samples:
        groups[tuple(sorted(set(sample.classes)))].append(sample)
    splits = {name: [] for name in ratios}
    for group in groups.values():
        rng.shuffle(group)
        cumulative = 0.0
        boundaries = []
        for ratio in list(ratios.values())[:-1]:
            cumulative += ratio
            boundaries.append(round(len(group) * cumulative))
        chunks = [group[:boundaries[0]], group[boundaries[0]:boundaries[1]], group[boundaries[1]:]]
        for name, chunk in zip(ratios, chunks):
            splits[name].extend(chunk)
    return splits


def materialize(splits: dict[str, list[Sample]], output: Path) -> None:
    for split, samples in splits.items():
        image_dir, label_dir = output / "images" / split, output / "labels" / split
        image_dir.mkdir(parents=True, exist_ok=True)
        label_dir.mkdir(parents=True, exist_ok=True)
        for sample in samples:
            stem = f"{sample.source}_{sample.sha256[:12]}_{sample.image.stem}"
            shutil.copy2(sample.image, image_dir / f"{stem}{sample.image.suffix.lower()}")
            shutil.copy2(sample.label, label_dir / f"{stem}.txt")


def main() -> int:
    args = parse_args()
    sources = args.source or [ROOT / "data/SDNET2018", ROOT / "data/MBDD2025", ROOT / "data/earthen_augmented"]
    ratios = {"train": args.train, "val": args.val, "test": args.test}
    if any(value < 0 for value in ratios.values()) or abs(sum(ratios.values()) - 1.0) > 1e-9:
        print("ERROR: split ratios must be non-negative and sum to 1", file=sys.stderr)
        return 2
    if args.output.exists() and any(args.output.iterdir()):
        if not args.overwrite:
            print(f"ERROR: output is not empty: {args.output}; pass --overwrite", file=sys.stderr)
            return 2
        shutil.rmtree(args.output)
    try:
        samples, issues = inspect_sources([path.resolve() for path in sources], args)
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    if not samples:
        print("ERROR: no valid labelled samples found; no output dataset was created", file=sys.stderr)
        print(json.dumps({"sources": [str(p) for p in sources], "issues": issues}, indent=2))
        return 3
    splits = split_samples(samples, ratios, args.seed)
    if any(not split for split in splits.values()):
        print("ERROR: at least one split is empty; add data or adjust ratios", file=sys.stderr)
        return 3
    materialize(splits, args.output)
    report = {
        "status": "COMPLETED", "seed": args.seed, "sources": [str(p) for p in sources],
        "valid_images": len(samples), "rejected_or_duplicate": len(issues),
        "splits": {name: len(items) for name, items in splits.items()},
        "class_instances": {CLASS_NAMES[key]: value for key, value in sorted(Counter(c for s in samples for c in s.classes).items())},
        "issues": issues,
    }
    (args.output / "dataset_report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    (args.output / "manifest.jsonl").write_text("\n".join(json.dumps({"split": split, "source": s.source, "original_image": str(s.image), "sha256": s.sha256}) for split, items in splits.items() for s in items) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
