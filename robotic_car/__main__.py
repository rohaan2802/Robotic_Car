"""Review a saved map, run a deterministic mapping demo, or query the car."""

import argparse
from pathlib import Path
import time

import numpy as np
import requests

from .client import COMMANDS, CarClient
from .mapping import OccupancyMap, Pose

SAVED_MAP = Path(__file__).resolve().parents[1] / "data/saved_occupancy.npy"


def demo_map():
    grid = OccupancyMap(120)
    for pose, distance in (
        (Pose(0, 0, 0), 28), (Pose(0, 0, 90), 20),
        (Pose(0, 0, 180), 24), (Pose(0, 0, 270), 30),
        (Pose(8, 4, 0), 20), (Pose(8, 4, 90), 16),
        (Pose(8, 4, 180), 32), (Pose(8, 4, 270), 34),
    ):
        grid.update(pose, distance)
    return grid


def show_map(grid, title, image_path):
    if image_path:
        import matplotlib
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 6), layout="constrained")
    image = grid.draw(ax, crop=True)
    ax.set_title(title)
    colorbar = fig.colorbar(image, ax=ax, ticks=[.16, .50, .82])
    colorbar.ax.set_yticklabels(["Free", "Unknown", "Occupied"])
    if image_path:
        fig.savefig(image_path, dpi=160)
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="mode", required=True)
    for name in ("review", "demo"):
        mode = sub.add_parser(name)
        mode.add_argument("--image", type=Path, help="Save a plot instead of opening a window.")
        if name == "review":
            mode.add_argument("--map", type=Path, default=SAVED_MAP)
        else:
            mode.add_argument("--save-map", type=Path)
    scan = sub.add_parser("scan", help="Update a map with one live range at a supplied pose.")
    scan.add_argument("--url", default="http://192.168.4.1")
    scan.add_argument("--pose", type=float, nargs=3, required=True, metavar=("X", "Y", "HEADING"))
    scan.add_argument("--map", type=Path, help="Start with an existing probability grid.")
    scan.add_argument("--save-map", type=Path, required=True, help="Destination NumPy map.")
    for name in ("telemetry", "pulse"):
        mode = sub.add_parser(name)
        mode.add_argument("--url", default="http://192.168.4.1")
        if name == "pulse":
            mode.add_argument("direction", choices=COMMANDS)
            mode.add_argument("--ms", type=int, default=250)

    args = parser.parse_args()
    try:
        if args.mode in ("review", "demo"):
            grid = OccupancyMap.load(args.map) if args.mode == "review" else demo_map()
            title = "Saved occupancy grid" if args.mode == "review" else "Synthetic range observations"
            free, unknown, occupied = grid.counts()
            print(f"{title} • {grid.grid.shape[0]} × {grid.grid.shape[1]} cells")
            print(f"Free: {free:,} | Unknown: {unknown:,} | Occupied: {occupied:,}")
            if getattr(args, "save_map", None):
                np.save(args.save_map, grid.grid)
            show_map(grid, title, args.image)
        else:
            client = CarClient(args.url)
            if args.mode == "scan":
                grid = OccupancyMap.load(args.map) if args.map else OccupancyMap()
                distance = client.distance()
                if not grid.update(Pose(*args.pose), distance):
                    raise ValueError("No usable range reading. The output map was not written.")
                np.save(args.save_map, grid.grid)
                print(f"Applied one {distance:g} cm observation at pose {tuple(args.pose)}.")
            elif args.mode == "telemetry":
                print(f"Distance (cm): {client.distance()}")
                print(client.attitude())
            else:
                # Require the maintained sketch before issuing movement requests.
                if args.direction != "S":
                    if client.request("/health").text.strip() != "robotic-car/1":
                        raise ValueError("The maintained watchdog firmware is required.")
                    try:
                        client.command(args.direction, args.ms)
                        time.sleep(args.ms / 1000)
                    finally:
                        client.command("S")
                else:
                    client.command("S")
                print(f"{args.direction}: request completed.")
    except (requests.RequestException, OSError, ValueError, TypeError, KeyError) as error:
        parser.exit(2, f"{error}\n")


if __name__ == "__main__":
    main()
