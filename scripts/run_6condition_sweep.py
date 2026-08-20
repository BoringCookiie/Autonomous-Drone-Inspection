#!/usr/bin/env python3
"""
run_6condition_sweep.py
Automated 6-Condition Matrix Evaluation Controller (Step 8).

Author: Autonomous UAV Inspection Team
Description:
    Runs the full 3x2 evaluation matrix across:
      Detector Backends (3): raw_vlm, rag_vlm, yolo
      Flight Strategies (2): single_pass, revisit
    Logs per-condition metrics (Precision, Recall, F1, Latency, Flight Distance) into JSON logs.
"""

import os
import json
import time
import argparse
import numpy as np

# 3x2 Evaluation Matrix Conditions (C1 - C6)
MATRIX_CONDITIONS = [
    {"id": "C1", "backend": "raw_vlm", "strategy": "single_pass", "desc": "Raw Zero-shot VLM + Lawnmower Baseline"},
    {"id": "C2", "backend": "raw_vlm", "strategy": "revisit",     "desc": "Raw Zero-shot VLM + Revisit Loop"},
    {"id": "C3", "backend": "rag_vlm", "strategy": "single_pass", "desc": "RAG-Grounded VLM + Lawnmower Baseline"},
    {"id": "C4", "backend": "rag_vlm", "strategy": "revisit",     "desc": "Proposed: RAG-Grounded VLM + Revisit Loop"},
    {"id": "C5", "backend": "yolo",    "strategy": "single_pass", "desc": "Supervised YOLOv11 + Lawnmower Baseline"},
    {"id": "C6", "backend": "yolo",    "strategy": "revisit",     "desc": "Supervised YOLOv11 + Revisit Loop"}
]


def run_sweep(output_dir: str, mock: bool = True):
    if not mock:
        raise RuntimeError(
            'The real six-condition runner is not implemented yet; use --mock only for pipeline smoke tests.'
        )
    os.makedirs(output_dir, exist_ok=True)
    print("=" * 70)
    print(" Executing 6-Condition Automated Evaluation Sweep")
    print("=" * 70)

    sweep_results = {}

    for cond in MATRIX_CONDITIONS:
        cid = cond["id"]
        backend = cond["backend"]
        strategy = cond["strategy"]

        print(f"\n[RUNNING {cid}] Backend: {backend:<8} | Strategy: {strategy:<11} ({cond['desc']})")

        if mock:
            # Simulate evaluation metrics based on experimental baseline expectations
            time.sleep(0.5)
            if cid == "C1": # raw_vlm, single_pass
                precision, recall, f1, latency, flight_m = 0.62, 0.58, 0.60, 480.0, 120.0
            elif cid == "C2": # raw_vlm, revisit
                precision, recall, f1, latency, flight_m = 0.68, 0.65, 0.66, 520.0, 142.5
            elif cid == "C3": # rag_vlm, single_pass
                precision, recall, f1, latency, flight_m = 0.78, 0.74, 0.76, 310.0, 120.0
            elif cid == "C4": # rag_vlm, revisit (PROPOSED BEST)
                precision, recall, f1, latency, flight_m = 0.91, 0.88, 0.89, 345.0, 138.0
            elif cid == "C5": # yolo, single_pass
                precision, recall, f1, latency, flight_m = 0.84, 0.80, 0.82, 35.0,  120.0
            elif cid == "C6": # yolo, revisit
                precision, recall, f1, latency, flight_m = 0.89, 0.86, 0.87, 45.0,  134.0

        cond_log = {
            "condition_id": cid,
            "backend": backend,
            "strategy": strategy,
            "description": cond["desc"],
            "metrics": {
                "precision": precision,
                "recall": recall,
                "f1_score": f1,
                "latency_ms_per_frame": latency,
                "total_flight_distance_m": flight_m
            }
        }

        sweep_results[cid] = cond_log

        print(f"  -> Precision: {precision:.2f} | Recall: {recall:.2f} | F1: {f1:.2f}")
        print(f"  -> Latency  : {latency:.1f} ms/frame | Flight Delta: {flight_m:.1f} m")

        # Save individual condition log
        cond_file = os.path.join(output_dir, f"{cid}_{backend}_{strategy}.json")
        with open(cond_file, 'w', encoding='utf-8') as f:
            json.dump(cond_log, f, indent=2)

    # Save aggregated summary
    summary_file = os.path.join(output_dir, "sweep_summary.json")
    with open(summary_file, 'w', encoding='utf-8') as f:
        json.dump(sweep_results, f, indent=2)

    print("\n" + "=" * 70)
    print(f"[SUCCESS] 6-Condition Sweep Completed. Summary logged to: {summary_file}")
    print("=" * 70)


def main():
    parser = argparse.ArgumentParser(description="Run 6-Condition Matrix Evaluation Sweep")
    parser.add_argument("--output-dir", default="results/sweeps", help="Directory to store JSON logs")
    parser.add_argument("--mock", action="store_true", help="Use simulated metrics run (never publish as results)")
    args = parser.parse_args()

    run_sweep(args.output_dir, args.mock)


if __name__ == '__main__':
    main()
