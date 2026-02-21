#!/usr/bin/env python3
"""
generate_map.py  (stdlib only — no numpy required)
"""
import os
import struct

# Map parameters
RESOLUTION = 0.05   # 5cm per pixel
MAP_SIZE_M = 10.0   # 10m x 10m world
ORIGIN_X = -5.0
ORIGIN_Y = -5.0
SIZE = int(MAP_SIZE_M / RESOLUTION)  # 200 x 200 pixels


def world_to_pixel(wx, wy):
    px = int((wx - ORIGIN_X) / RESOLUTION)
    py = int((wy - ORIGIN_Y) / RESOLUTION)
    return px, py


def add_obstacle(img, wx, wy, width_m, height_m):
    px, py = world_to_pixel(wx, wy)
    pw = int(width_m / RESOLUTION)
    ph = int(height_m / RESOLUTION)
    x_start = max(0, px - pw // 2)
    x_end = min(SIZE, px + pw // 2)
    y_start = max(0, py - ph // 2)
    y_end = min(SIZE, py + ph // 2)
    for row in range(SIZE - y_end, SIZE - y_start):
        for col in range(x_start, x_end):
            img[row * SIZE + col] = 0


def generate_map():
    # All free space (254 = white)
    img = bytearray([254] * (SIZE * SIZE))

    # Border walls
    border = 4
    for col in range(SIZE):
        for r in range(border):
            img[r * SIZE + col] = 0
            img[(SIZE - 1 - r) * SIZE + col] = 0
    for row in range(SIZE):
        for c in range(border):
            img[row * SIZE + c] = 0
            img[row * SIZE + (SIZE - 1 - c)] = 0

    # Obstacles matching Gazebo world
    add_obstacle(img, 1.0,  0.5,  0.5, 0.5)
    add_obstacle(img, -0.5, 1.5,  0.5, 0.8)
    add_obstacle(img, 1.5,  -0.5, 0.6, 0.5)

    output_dir = os.path.dirname(os.path.abspath(__file__))
    pgm_path = os.path.join(output_dir, 'map.pgm')

    with open(pgm_path, 'wb') as f:
        f.write(f'P5\n{SIZE} {SIZE}\n255\n'.encode())
        f.write(bytes(img))

    print(f'Map saved: {pgm_path}  ({SIZE}x{SIZE} px, {MAP_SIZE_M}m x {MAP_SIZE_M}m)')

    yaml_path = os.path.join(output_dir, 'map.yaml')
    with open(yaml_path, 'w') as f:
        f.write(f"""image: map.pgm
resolution: {RESOLUTION}
origin: [{ORIGIN_X}, {ORIGIN_Y}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
""")
    print(f'Map YAML saved: {yaml_path}')
    return pgm_path, yaml_path


if __name__ == '__main__':
    generate_map()
