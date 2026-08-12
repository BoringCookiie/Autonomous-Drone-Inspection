#!/usr/bin/env python3
"""
generate_table4_figure6.py
Results Aggregator & Figure Generator for Earthen Heritage Inspection Benchmark.

Author: Autonomous UAV Inspection Team
Description:
    Reads 6-condition sweep JSON logs, formats Table 4 (Precision, Recall, F1, Latency, Flight-Δ),
    and generates Figure 6 (Accuracy vs. Latency Trade-off plot saved to `results/figure6_accuracy_latency.png`).
"""

import os
import json
import argparse


def generate_table4_and_figure6(input_dir: str, output_dir: str):
    summary_file = os.path.join(input_dir, "sweep_summary.json")

    if not os.path.exists(summary_file):
        print(f"[ERROR] Sweep summary file {summary_file} not found. Run `run_6condition_sweep.py` first.")
        return

    with open(summary_file, 'r', encoding='utf-8') as f:
        data = json.load(f)

    os.makedirs(output_dir, exist_ok=True)

    # 1. Generate & Print Table 4 ASCII & JSON Summary
    print("\n" + "=" * 90)
    print(" TABLE 4: SIX-CONDITION EXPERIMENTAL EVALUATION RESULTS ")
    print("=" * 90)
    header = f"{'ID':<4} | {'Backend':<9} | {'Strategy':<11} | {'Precision':<9} | {'Recall':<7} | {'F1':<6} | {'Latency (ms)':<12} | {'Flight (m)':<10}"
    print(header)
    print("-" * 90)

    table4_rows = []

    for cid, info in data.items():
        m = info["metrics"]
        row_str = f"{cid:<4} | {info['backend']:<9} | {info['strategy']:<11} | {m['precision']:<9.2f} | {m['recall']:<7.2f} | {m['f1_score']:<6.2f} | {m['latency_ms_per_frame']:<12.1f} | {m['total_flight_distance_m']:<10.1f}"
        print(row_str)

        table4_rows.append({
            "condition": cid,
            "backend": info["backend"],
            "strategy": info["strategy"],
            "precision": m["precision"],
            "recall": m["recall"],
            "f1_score": m["f1_score"],
            "latency_ms": m["latency_ms_per_frame"],
            "flight_distance_m": m["total_flight_distance_m"]
        })

    print("=" * 90)

    # Save Table 4 JSON
    table4_path = os.path.join(output_dir, "table4_summary.json")
    with open(table4_path, 'w', encoding='utf-8') as f:
        json.dump(table4_rows, f, indent=2)
    print(f"[SUCCESS] Saved Table 4 summary to {table4_path}")

    # 2. Plot Figure 6 (Accuracy vs Latency Trade-off)
    try:
        import matplotlib.pyplot as plt

        cids = [r["condition"] for r in table4_rows]
        f1_scores = [r["f1_score"] for r in table4_rows]
        latencies = [r["latency_ms"] for r in table4_rows]
        labels = [f"{r['condition']} ({r['backend']})" for r in table4_rows]

        plt.figure(figsize=(9, 6))
        scatter = plt.scatter(latencies, f1_scores, c=range(len(cids)), cmap='viridis', s=180, edgecolors='black', zorder=3)

        for i, txt in enumerate(labels):
            plt.annotate(txt, (latencies[i] + 10, f1_scores[i] - 0.01), fontsize=10, fontweight='bold')

        plt.title("Figure 6: Defect Detection F1-Score vs. Inference Latency (ms)", fontsize=13, fontweight='bold')
        plt.xlabel("Latency per Frame (ms)", fontsize=11)
        plt.ylabel("F1-Score", fontsize=11)
        plt.grid(True, linestyle='--', alpha=0.6)
        plt.ylim(0.5, 1.0)

        fig6_path = os.path.join(output_dir, "figure6_accuracy_latency.png")
        plt.savefig(fig6_path, dpi=300, bbox_inches='tight')
        plt.close()
        print(f"[SUCCESS] Saved Figure 6 plot to {fig6_path}")

    except ImportError:
        print("[WARNING] matplotlib not installed. Skipping Figure 6 png generation.")


def main():
    parser = argparse.ArgumentParser(description="Generate Table 4 & Figure 6")
    parser.add_argument("--input-dir", default="results/sweeps", help="Input directory containing sweep summary")
    parser.add_argument("--output-dir", default="results", help="Output directory for table4 and figure6")
    args = parser.parse_args()

    generate_table4_and_figure6(args.input_dir, args.output_dir)


if __name__ == '__main__':
    main()
