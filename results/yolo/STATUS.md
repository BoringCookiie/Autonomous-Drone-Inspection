# YOLO11 experiment status

**Status: 20-EPOCH SUPERVISED BASELINE COMPLETED**

A real 20-epoch CPU training run completed on 2026-08-15 using the prepared
MBDD2025 subset (3,798 train / 814 validation / 812 test images), YOLO11s,
640 px images, batch size 4, and seed 42. Training took 10.366 hours.

Real held-out test metrics:

| Metric | Value |
|---|---:|
| Precision | 0.668022 |
| Recall | 0.559108 |
| F1 | 0.608732 |
| mAP@50 | 0.601448 |
| mAP@50-95 | 0.250937 |
| CPU inference | 37.129 ms/image |
| CPU throughput | 26.933 FPS |

The actual best weights were copied to `models/yolo/yolo_earthen_v11.pt` and
the complete machine-readable report is in `metrics.json` locally. Generated
weights, datasets, and run artifacts are intentionally Git-ignored.

Per-class test mAP@50 was 0.395 for `structural_crack`, 0.599 for
`surface_erosion`, and 0.810 for `moisture_stain`. This is the project's real
supervised baseline, not a simulated result. Structural-crack recall remains the
main weakness, and qualitative review of the aggressive near-duplicate filtering
is still required before claiming production readiness.
