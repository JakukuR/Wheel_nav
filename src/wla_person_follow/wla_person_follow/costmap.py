"""Six-meter robot-centered obstacle grid at ten-centimeter resolution."""
import cv2
import numpy as np


class LocalCostmap:
    def __init__(self, size=6.0, resolution=.10, inflation=.45):
        self.size = float(size)
        self.resolution = float(resolution)
        self.cells = int(round(size / resolution))
        self.inflation = float(inflation)
        self.occupied = np.zeros((self.cells, self.cells), dtype=np.uint8)
        self.raw = np.empty((0, 3), dtype=np.float32)

    def update(self, points):
        self.raw = np.asarray(points, dtype=np.float32)
        grid = np.zeros_like(self.occupied)
        if self.raw.size:
            x = self.raw[:, 0]
            y = self.raw[:, 1]
            ix = np.floor((x + self.size/2) / self.resolution).astype(int)
            iy = np.floor((y + self.size/2) / self.resolution).astype(int)
            valid = (ix >= 0) & (ix < self.cells) & (iy >= 0) & (iy < self.cells)
            grid[iy[valid], ix[valid]] = 1
        radius = int(np.ceil(self.inflation / self.resolution))
        yy, xx = np.ogrid[-radius:radius+1, -radius:radius+1]
        kernel = ((xx*xx + yy*yy) <= radius*radius).astype(np.uint8)
        self.occupied = cv2.dilate(grid, kernel)

    def front_distance(self, half_width=.34):
        if not self.raw.size:
            return float('inf')
        x, y = self.raw[:, 0], self.raw[:, 1]
        forward = x[(x > 0) & (np.abs(y) < half_width)]
        return float(np.min(forward)) if forward.size else float('inf')

    def sector_blocked(self, side):
        if not self.raw.size:
            return False
        x, y = self.raw[:, 0], self.raw[:, 1]
        return bool(np.any((x > .20) & (x < 1.5) & (y * side > .05) & (y * side < .9)))

    def collision_at(self, x, y):
        ix = int(np.floor((x + self.size/2) / self.resolution))
        iy = int(np.floor((y + self.size/2) / self.resolution))
        if not (0 <= ix < self.cells and 0 <= iy < self.cells):
            return True
        return bool(self.occupied[iy, ix])
