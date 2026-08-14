# Data track and YOLO11 supervised baseline

## Current factual status

At implementation time, `SDNET2018`, `MBDD2025`, and `earthen_augmented` contained only
repository placeholders. There were no annotations, prepared splits, trained weights, or
YOLO metrics. Consequently the experiment status is **NOT RUN**; no metric or model is
simulated.

## Reproducible workflow

1. Download each dataset from its authoritative source and retain its license/provenance.
2. Create or convert genuine bounding boxes to canonical YOLO labels. Review mappings using
   `data/class_mapping.yaml`; do not map a generic defect to a class without evidence.
   For an existing YOLO dataset with documented class names, remap into a new directory:

   ```bash
   python scripts/remap_yolo_labels.py --labels SOURCE/labels --source-yaml SOURCE/data.yaml --output SOURCE/labels_canonical
   ```

   The converter fails on every unmapped class and never overwrites source annotations.
3. Put paired images and labels below their source folders as described in `data/README.md`.
4. Install the isolated training dependencies and prepare the dataset:

   ```bash
   python -m pip install -r requirements-yolo.txt
   python scripts/prepare_yolo_dataset.py --seed 42
   ```

5. Review `data/yolo/dataset_report.json` and `manifest.jsonl`. Resolve rejected annotations
   at source, then rebuild with `--overwrite`. Visually inspect a representative sample of
   every class and split before training.
6. Train and evaluate on the untouched test split:

   ```bash
   python scripts/train_yolov11.py --epochs 100 --batch 16 --imgsz 640 --device 0 --seed 42
   ```

The trainer applies conservative online augmentation from `data/augmentation.yaml`, saves
Ultralytics training/validation plots under `results/yolo/runs`, copies only the actual best
checkpoint to `models/yolo/yolo_earthen_v11.pt`, evaluates the test split, and writes real
precision, recall, F1, mAP50, mAP50-95, inference time, and FPS to
`results/yolo/metrics.json`. Confusion matrices and labelled prediction plots provide the
qualitative review material for false positives and false negatives.

Use `--device cpu` where CUDA is unavailable. A GPU is recommended but not required. The
default checkpoint name is the Ultralytics YOLO11 spelling, `yolo11s.pt` (not
`yolov11s.pt`). Downloading that pretrained checkpoint requires network access on first run.

## ROS integration

Select the backend with:

```bash
ros2 launch uas_earthen_inspection inspection_pipeline.launch.py detector_backend:=yolo
```

The node reads `yolo_weights_path`, `yolo_confidence_threshold`, `yolo_device`, and
`yolo_image_size` from `inspection_params.yaml`. It accepts only the canonical three-class
model. Missing, incompatible, or unloadable weights produce a clear error and no fake
detections; the process remains alive so configuration can be diagnosed.
