#!/usr/bin/env python3
"""
generate_map.py
---------------
Generates a Nav2-compatible occupancy grid (PGM + YAML) that matches
the Gazebo world file.  Every wall and static obstacle in the .world
must have a corresponding add_box / add_cylinder call here so the
global costmap planner knows about them.

Run:  python3 generate_map.py
"""

import os
import math


class MapGenerator:
    def __init__(self, size_m=10.0, resolution=0.05):
        self.res = resolution
        self.size_px = int(size_m / resolution)
        self.origin_m = -(size_m / 2.0)
        # 254 = Free, 0 = Occupied, 205 = Unknown
        self.grid = bytearray([254] * (self.size_px * self.size_px))

    def _to_pixel(self, x_m, y_m):
        px = int((x_m - self.origin_m) / self.res)
        py = int((y_m - self.origin_m) / self.res)
        return px, py

    def add_box(self, cx, cy, w, h):
        """Mark a box centred at (cx, cy) with width w and height h as occupied."""
        px, py = self._to_pixel(cx, cy)
        pw, ph = int(w / self.res), int(h / self.res)
        x0 = max(0, px - pw // 2)
        x1 = min(self.size_px, px + pw // 2)
        y0 = max(0, py - ph // 2)
        y1 = min(self.size_px, py + ph // 2)
        for r in range(y0, y1):
            row = (self.size_px - 1 - r) * self.size_px
            for c in range(x0, x1):
                self.grid[row + c] = 0

    def add_cylinder(self, cx, cy, radius):
        """Mark a cylinder centred at (cx, cy) as occupied."""
        px, py = self._to_pixel(cx, cy)
        pr = int(radius / self.res)
        for r in range(max(0, py - pr), min(self.size_px, py + pr + 1)):
            row = (self.size_px - 1 - r) * self.size_px
            for c in range(max(0, px - pr), min(self.size_px, px + pr + 1)):
                if (c - px) ** 2 + (r - py) ** 2 <= pr ** 2:
                    self.grid[row + c] = 0

    def save(self, folder, name="map"):
        pgm_fn = f"{name}.pgm"
        yaml_fn = f"{name}.yaml"
        with open(os.path.join(folder, pgm_fn), 'wb') as f:
            f.write(f"P5\n{self.size_px} {self.size_px}\n255\n".encode())
            f.write(self.grid)
        with open(os.path.join(folder, yaml_fn), 'w') as f:
            f.write(f"image: {pgm_fn}\n")
            f.write(f"resolution: {self.res}\n")
            f.write(f"origin: [{self.origin_m}, {self.origin_m}, 0.0]\n")
            f.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
        print(f"Map saved to {folder}/{pgm_fn} and {yaml_fn}")


if __name__ == "__main__":
    gen = MapGenerator(size_m=10.0, resolution=0.05)

    # ── Outer walls ──
    gen.add_box(0,     5.0,  10.15, 0.15)   # north
    gen.add_box(0,    -5.0,  10.15, 0.15)   # south
    gen.add_box(-5.0,  0,     0.15, 10.0)   # west
    gen.add_box(5.0,   0,     0.15, 10.0)   # east

    # ── Interior partitions ──
    gen.add_box(-3.0, -1.5,  4.0,  0.15)    # lower horizontal
    gen.add_box(-3.0,  1.5,  4.0,  0.15)    # upper horizontal
    gen.add_box(1.5,  -2.0,  0.15, 6.0)     # right vertical (lower) — ends at y=+1
    gen.add_box(1.5,   4.5,  0.15, 1.0)     # right vertical (upper stub) — starts at y=+4

    # ── Obstacles ──
    gen.add_box(0.0,      -3.0,  0.8, 0.5)           # table
    gen.add_cylinder(3.5,   0.0,  0.25)               # barrel_1
    gen.add_box(-0.5,      3.5,  0.5, 0.5)            # crate_1
    gen.add_cylinder(3.5,  -3.0,  0.25)               # barrel_2
    gen.add_box(-1.8,      0.0,  0.4, 0.4)            # crate_2
    gen.add_box(-0.5,     -4.0,  0.6, 0.3)            # box_south
    gen.add_cylinder(-1.3, -2.8,  0.2)                 # barrel_3

    script_dir = os.path.dirname(os.path.abspath(__file__))
    gen.save(script_dir)