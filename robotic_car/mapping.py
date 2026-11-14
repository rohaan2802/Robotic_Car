"""Centimeter-grid occupancy mapping for one forward range observation at a time."""

from dataclasses import dataclass
import math

import numpy as np

MAX_RANGE_CM = 100


@dataclass(frozen=True)
class Pose:
    x: float = 0
    y: float = 0
    heading: float = 0


def heading_delta(current, target):
    """Return the signed shortest turn from current to target in degrees."""
    return (target - current + 180) % 360 - 180


class OccupancyMap:
    def __init__(self, size=500):
        if type(size) is not int or size < 2:
            raise ValueError("Grid size must be an integer of at least two.")
        self.grid = np.full((size, size), 0.5)

    @classmethod
    def load(cls, path):
        grid = np.load(path, allow_pickle=False)
        if (grid.ndim != 2 or grid.shape[0] != grid.shape[1]
                or grid.dtype.kind not in "fiu" or not np.isfinite(grid).all()
                or np.any((grid < 0) | (grid > 1))):
            raise ValueError("Expected a square grid of finite probabilities from 0 to 1.")
        result = cls(grid.shape[0])
        result.grid = grid.astype(float)
        return result

    def cell(self, x, y):
        center = self.grid.shape[0] // 2
        return math.floor(y) + center, math.floor(x) + center

    def update(self, pose, distance):
        """Apply free evidence before a valid hit and occupied evidence at the hit."""
        if distance is None or not math.isfinite(distance) or not 0 < distance <= MAX_RANGE_CM:
            return False
        if not all(map(math.isfinite, (pose.x, pose.y, pose.heading))):
            raise ValueError("Pose must contain finite coordinates and heading.")

        angle = math.radians(pose.heading)
        dx, dy = math.cos(angle), math.sin(angle)
        # Remove floating-point drift on exact cardinal headings.
        dx = 0.0 if abs(dx) < 1e-15 else dx
        dy = 0.0 if abs(dy) < 1e-15 else dy

        def project(r):
            return self.cell(pose.x + r * dx, pose.y + r * dy)

        hit = project(distance)
        steps = math.ceil(distance * 2)
        free = {project(distance * i / steps) for i in range(steps)} - {hit}
        size = self.grid.shape[0]
        for cells, odds_ratio in ((free, 1/9), ((hit,), 9)):
            for row, col in cells:
                if 0 <= row < size and 0 <= col < size:
                    prior = self.grid[row, col]
                    posterior = odds_ratio * prior / (1 - prior + odds_ratio * prior)
                    self.grid[row, col] = min(0.99, max(0.01, posterior))
        return True

    def counts(self):
        free = int(np.count_nonzero(self.grid <= 0.35))
        occupied = int(np.count_nonzero(self.grid >= 0.65))
        return free, self.grid.size - free - occupied, occupied

    def draw(self, axis, *, crop=False):
        from matplotlib.colors import BoundaryNorm, ListedColormap
        palette = ListedColormap(["#f4a444", "#142537", "#40c9b8"])
        norm = BoundaryNorm([-0.01, 0.35000001, 0.65, 1.01], 3)
        n = self.grid.shape[0]
        center = n // 2
        image = axis.imshow(self.grid, origin="lower", cmap=palette, norm=norm,
                            extent=(-center, n-center, -center, n-center),
                            interpolation="nearest")
        if crop:
            rows, cols = np.where((self.grid <= 0.35) | (self.grid >= 0.65))
            if rows.size:
                axis.set_xlim(cols.min()-center-5, cols.max()-center+6)
                axis.set_ylim(rows.min()-center-5, rows.max()-center+6)
        axis.set(xlabel="x (cm)", ylabel="y (cm)")
        return image
