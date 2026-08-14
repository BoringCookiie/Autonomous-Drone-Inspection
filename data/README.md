# Datasets Directory Manifest

This directory houses dataset assets used for YOLOv11 training and evaluation of zero-shot VLM defect detection.

## Structure
- `SDNET2018/`: Concrete and masonry crack dataset used for pre-training/fine-tuning.
- `MBDD2025/`: Masonry & Building Defect Dataset 2025.
- `earthen_augmented/`: Domain-adapted synthetic and augmented earthen heritage defect crops (cracks, erosion, moisture stains).
- `evaluation_set/`: 30–50 hand-labeled high-resolution ground truth evaluation frames captured from Gazebo mudbrick world and real-world earthen walls.

## Canonical classes

The supervised detector uses exactly three ontology-aligned classes:

| ID | Class |
|---:|---|
| 0 | `structural_crack` |
| 1 | `surface_erosion` |
| 2 | `moisture_stain` |

Source aliases are documented in `class_mapping.yaml`. Source labels must be converted
and manually verified against that mapping; the repository does not fabricate missing
bounding boxes. Note that classification-only SDNET2018 files are not directly usable
for object detection until real bounding boxes have been annotated.

## Expected source layout

Each source must already use paired YOLO labels. Both co-located labels and the standard
`images/...` + `labels/...` layout are accepted:

```text
data/SDNET2018/
├── images/
│   └── sample.jpg
└── labels/
    └── sample.txt
```

Large binary files are excluded from Git. After placing and verifying real data, run:

```bash
python scripts/prepare_yolo_dataset.py
```

The command rejects corrupt/small images, missing or malformed annotations, out-of-range
classes and boxes, empty labels (unless `--allow-empty` is explicit), exact duplicates, and
near-duplicates using a configurable perceptual-hash distance.
It creates `data/yolo/{images,labels}/{train,val,test}`, a provenance manifest, and a JSON
quality report. Deduplication occurs before the seeded stratified split to prevent leakage.

Use repeated `--source` arguments for other roots. `--overwrite` is required to replace an
existing prepared dataset. Never use it before preserving a dataset that needs review.
