#!/usr/bin/env python3
"""Remap genuine YOLO annotations to the three canonical project classes."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--labels", type=Path, required=True, help="Source labels directory")
    parser.add_argument("--source-yaml", type=Path, required=True, help="Source YOLO YAML containing names")
    parser.add_argument("--mapping", type=Path, default=ROOT / "data/class_mapping.yaml")
    parser.add_argument("--output", type=Path, required=True, help="New labels directory (never in-place)")
    parser.add_argument("--drop-class", action="append", default=[], help="Documented source class to omit; repeatable")
    args = parser.parse_args()
    try:
        import yaml
    except ImportError:
        print("ERROR: PyYAML is required", file=sys.stderr)
        return 2
    if args.output.resolve() == args.labels.resolve() or args.output.exists():
        print("ERROR: output must be a new directory; source labels are never overwritten", file=sys.stderr)
        return 2
    source_cfg = yaml.safe_load(args.source_yaml.read_text(encoding="utf-8")) or {}
    mapping_cfg = yaml.safe_load(args.mapping.read_text(encoding="utf-8")) or {}
    raw_names = source_cfg.get("names", {})
    source_names = {int(key): value for key, value in raw_names.items()} if isinstance(raw_names, dict) else dict(enumerate(raw_names))
    canonical = {name: int(index) for index, name in mapping_cfg.get("names", {}).items()}
    aliases = mapping_cfg.get("aliases", {})
    dropped_names = {name.strip().lower() for name in args.drop_class}
    id_map = {}
    dropped_ids = set()
    for source_id, source_name in source_names.items():
        normalized_name = str(source_name).strip().lower()
        if normalized_name in dropped_names:
            dropped_ids.add(source_id)
            continue
        target_name = aliases.get(normalized_name)
        if target_name in canonical:
            id_map[source_id] = canonical[target_name]
    unmapped = sorted(set(source_names) - set(id_map) - dropped_ids)
    if unmapped:
        details = ", ".join(f"{idx}:{source_names[idx]}" for idx in unmapped)
        print(f"ERROR: unmapped source classes ({details}); review data/class_mapping.yaml", file=sys.stderr)
        return 3
    files = sorted(args.labels.rglob("*.txt"))
    if not files:
        print("ERROR: no source label files found", file=sys.stderr)
        return 3
    for source in files:
        output_lines = []
        for line_no, line in enumerate(source.read_text(encoding="utf-8").splitlines(), 1):
            if not line.strip():
                continue
            fields = line.split()
            try:
                source_id = int(fields[0])
            except (ValueError, IndexError):
                print(f"ERROR: malformed class id at {source}:{line_no}", file=sys.stderr)
                return 3
            if source_id in dropped_ids:
                continue
            if source_id not in id_map:
                print(f"ERROR: unknown class id {source_id} at {source}:{line_no}", file=sys.stderr)
                return 3
            fields[0] = str(id_map[source_id])
            output_lines.append(" ".join(fields))
        target = args.output / source.relative_to(args.labels)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text("\n".join(output_lines) + ("\n" if output_lines else ""), encoding="utf-8")
    print(f"Remapped {len(files)} label files to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
