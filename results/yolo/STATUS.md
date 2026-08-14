# YOLO11 experiment status

**Status: SMOKE TEST COMPLETED — FINAL TRAINING NOT RUN**

A real two-epoch CPU smoke test completed on 2026-08-14 using the prepared
MBDD2025 subset (3,798 train / 814 validation / 812 test images), YOLO11s,
640 px images, batch size 4, and seed 42.

Real held-out test metrics:

| Metric | Value |
|---|---:|
| Precision | 0.188370 |
| Recall | 0.186959 |
| F1 | 0.187662 |
| mAP@50 | 0.117543 |
| mAP@50-95 | 0.035685 |
| CPU inference | 41.128 ms/image |
| CPU throughput | 24.314 FPS |

The actual best weights were copied to `models/yolo/yolo_earthen_v11.pt` and
the complete machine-readable report is in `metrics.json` locally. Generated
weights, datasets, and run artifacts are intentionally Git-ignored.

These values demonstrate that the end-to-end pipeline works; they are not
final model performance. Two epochs are insufficient for convergence. A full
training run and qualitative review of the aggressive near-duplicate filtering
are still required before the model can be presented as the supervised baseline.
