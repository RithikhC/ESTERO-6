"""Turn hotspots.csv into a contamination density heatmap (PNG).

Usage:
  ros2 run bayani_core make_heatmap --csv ~/.bayani/hotspots.csv --out heatmap.png
  ros2 run bayani_core make_heatmap --csv hotspots.csv --map my_map.yaml   # overlay on the SLAM map
"""
import argparse
import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402


def load_points(path):
    pts = []
    with open(os.path.expanduser(path)) as f:
        for row in csv.DictReader(f):
            pts.append((float(row["x_m"]), float(row["y_m"]), float(row.get("score", 1.0))))
    return np.array(pts)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", default="~/.bayani/hotspots.csv")
    ap.add_argument("--map", default="", help="nav2/cartographer map .yaml to draw underneath")
    ap.add_argument("--sigma", type=float, default=0.8, help="blob radius in metres")
    ap.add_argument("--out", default="bayani_heatmap.png")
    a = ap.parse_args()

    pts = load_points(a.csv)
    if len(pts) == 0:
        raise SystemExit("No hotspots in CSV yet.")

    fig, ax = plt.subplots(figsize=(10, 6), dpi=150)
    if a.map:
        meta = yaml.safe_load(open(a.map))
        img = plt.imread(os.path.join(os.path.dirname(a.map), meta["image"]))
        res, (ox, oy) = meta["resolution"], meta["origin"][:2]
        extent = [ox, ox + img.shape[1] * res, oy, oy + img.shape[0] * res]
        ax.imshow(img, cmap="gray", extent=extent, origin="upper")
    else:
        pad = 3 * a.sigma
        extent = [pts[:, 0].min() - pad, pts[:, 0].max() + pad, pts[:, 1].min() - pad, pts[:, 1].max() + pad]

    xs = np.linspace(extent[0], extent[1], 400)
    ys = np.linspace(extent[2], extent[3], 300)
    gx, gy = np.meshgrid(xs, ys)
    z = np.zeros_like(gx)
    for x, y, s in pts:
        z += s * np.exp(-((gx - x) ** 2 + (gy - y) ** 2) / (2 * a.sigma ** 2))
    z = np.ma.masked_less(z, 0.05)
    hm = ax.imshow(z, extent=extent, origin="lower", cmap="YlOrRd", alpha=0.75)
    ax.scatter(pts[:, 0], pts[:, 1], s=30, c="black", marker="D", label="hotspot")
    for i, (x, y, _) in enumerate(pts, 1):
        ax.annotate(f"H{i}", (x, y), textcoords="offset points", xytext=(6, 6), fontsize=8)
    fig.colorbar(hm, ax=ax, label="contamination density")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Bayani-6 contamination map")
    ax.set_aspect("equal")
    fig.tight_layout()
    fig.savefig(a.out)
    print(f"saved {a.out} ({len(pts)} hotspots)")


if __name__ == "__main__":
    main()
