#!/usr/bin/env python3
"""plot_trajectory.py — top-down + altitude profile of a flight from a bag's
local_position.csv export (produced by analyze_rosbags.py --export-csv)."""
from __future__ import annotations

import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def main() -> None:
    csv_path = Path(sys.argv[1])
    out_png = Path(sys.argv[2]) if len(sys.argv) > 2 else csv_path.parent / "trajectory.png"
    rows = list(csv.DictReader(csv_path.open()))
    t = [float(r["time_s"]) for r in rows]
    x = [float(r["x_m"]) for r in rows]
    y = [float(r["y_m"]) for r in rows]
    z = [float(r["z_m"]) for r in rows]

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    sc = axes[0].scatter(x, y, c=t, s=4, cmap="viridis")
    axes[0].set_xlabel("x [m]"); axes[0].set_ylabel("y [m]")
    axes[0].set_title("Top-down (color = time)")
    axes[0].axis("equal")
    plt.colorbar(sc, ax=axes[0])

    axes[1].plot(t, z, lw=1)
    axes[1].set_xlabel("t [s]"); axes[1].set_ylabel("z [m]")
    axes[1].set_title("Altitude profile")

    axes[2].plot(t, x, lw=1, label="x")
    axes[2].plot(t, y, lw=1, label="y")
    axes[2].set_xlabel("t [s]"); axes[2].set_ylabel("[m]")
    axes[2].legend(); axes[2].set_title("X/Y vs time")

    fig.tight_layout()
    fig.savefig(out_png, dpi=140)
    print(f"wrote {out_png}")


if __name__ == "__main__":
    main()
