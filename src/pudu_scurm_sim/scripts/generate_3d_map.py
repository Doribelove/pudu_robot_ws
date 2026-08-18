#!/usr/bin/env python3
"""Extrude a ROS occupancy PGM into a deterministic XYZ/I PCD prior map."""

import argparse
from pathlib import Path


def read_pgm(path):
    with path.open("rb") as stream:
        tokens = []
        while len(tokens) < 4:
            line = stream.readline()
            if not line:
                raise ValueError(f"Incomplete PGM header: {path}")
            line = line.split(b"#", 1)[0]
            tokens.extend(line.split())
        magic, width, height, maximum = tokens[:4]
        if magic != b"P5" or int(maximum) > 255:
            raise ValueError("Only 8-bit binary P5 PGM maps are supported")
        width = int(width)
        height = int(height)
        pixels = stream.read(width * height)
        if len(pixels) != width * height:
            raise ValueError(f"Incomplete PGM pixel data: {path}")
        return width, height, pixels


def parse_map_yaml(path):
    values = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        values[key.strip()] = value.strip()
    image = values["image"]
    image_path = Path(image)
    if not image_path.is_absolute():
        image_path = path.parent / image_path
    origin = [float(item.strip()) for item in values["origin"].strip("[]").split(",")]
    return image_path, float(values["resolution"]), origin


def create_points(width, height, pixels, resolution, origin, xy_step, z_step, wall_height):
    occupied_threshold = 89  # ROS occupied_thresh=0.65 with negate=0.

    def occupied(row, column):
        return pixels[row * width + column] <= occupied_threshold

    points = []
    cell_stride = max(1, round(xy_step / resolution))
    for row in range(0, height, cell_stride):
        for column in range(0, width, cell_stride):
            if not occupied(row, column):
                continue
            # Retain occupied boundaries; filled wall interiors add little to ICP.
            boundary = any(
                neighbor_row < 0 or neighbor_row >= height
                or neighbor_column < 0 or neighbor_column >= width
                or not occupied(neighbor_row, neighbor_column)
                for neighbor_row, neighbor_column in (
                    (row - 1, column), (row + 1, column),
                    (row, column - 1), (row, column + 1)))
            if not boundary:
                continue
            x = origin[0] + (column + 0.5) * resolution
            y = origin[1] + (height - row - 0.5) * resolution
            z = z_step * 0.5
            while z <= wall_height:
                points.append((x, y, z, 100.0))
                z += z_step

    # A sparse floor stabilizes z/roll/pitch without overwhelming wall geometry.
    floor_stride = max(1, round(0.30 / resolution))
    for row in range(0, height, floor_stride):
        for column in range(0, width, floor_stride):
            pixel = pixels[row * width + column]
            if pixel >= 250:
                x = origin[0] + (column + 0.5) * resolution
                y = origin[1] + (height - row - 0.5) * resolution
                points.append((x, y, 0.0, 30.0))
    return points


def write_pcd(path, points):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii") as stream:
        stream.write("# .PCD v0.7 - Point Cloud Data file format\n")
        stream.write("VERSION 0.7\n")
        stream.write("FIELDS x y z intensity normal_x normal_y normal_z curvature\n")
        stream.write("SIZE 4 4 4 4 4 4 4 4\n")
        stream.write("TYPE F F F F F F F F\n")
        stream.write("COUNT 1 1 1 1 1 1 1 1\n")
        stream.write(f"WIDTH {len(points)}\nHEIGHT 1\n")
        stream.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        stream.write(f"POINTS {len(points)}\nDATA ascii\n")
        for x, y, z, intensity in points:
            stream.write(
                f"{x:.4f} {y:.4f} {z:.4f} {intensity:.1f} 0 0 0 0\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("map_yaml", type=Path)
    parser.add_argument("output_pcd", type=Path)
    parser.add_argument("--xy-step", type=float, default=0.10)
    parser.add_argument("--z-step", type=float, default=0.10)
    parser.add_argument("--wall-height", type=float, default=1.60)
    args = parser.parse_args()

    image_path, resolution, origin = parse_map_yaml(args.map_yaml)
    width, height, pixels = read_pgm(image_path)
    points = create_points(
        width, height, pixels, resolution, origin,
        args.xy_step, args.z_step, args.wall_height)
    write_pcd(args.output_pcd, points)
    print(f"Wrote {len(points)} points to {args.output_pcd}")


if __name__ == "__main__":
    main()
