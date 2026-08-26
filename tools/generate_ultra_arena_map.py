#!/usr/bin/env python3
"""Generate a 500 m x 400 m Arena occupancy map at 5 cm resolution.

The layout is deliberately synthetic: an open campus/plaza surrounds four
building blocks, with wide halls, narrow corridors, room partitions, doors,
and warehouse aisles. Coordinates are in metres with origin at the southwest
corner of the map.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


RESOLUTION = 0.05
WIDTH_M = 500.0
HEIGHT_M = 400.0
FREE = 254
UNKNOWN = 205
OCCUPIED = 0


class Layout:
    def __init__(self):
        self.width = round(WIDTH_M / RESOLUTION)
        self.height = round(HEIGHT_M / RESOLUTION)
        self.image = np.full((self.height, self.width), FREE, dtype=np.uint8)
        self.wall_cells = 6  # 0.30 m structural walls
        self.door_width_m = 2.0
        self.doors: list[dict] = []

    def point(self, x: float, y: float) -> tuple[int, int]:
        return round(x / RESOLUTION), round(HEIGHT_M / RESOLUTION - y / RESOLUTION)

    def segment(self, x0, y0, x1, y1, thickness=None):
        thickness = self.wall_cells if thickness is None else thickness
        cv2.line(self.image, self.point(x0, y0), self.point(x1, y1), OCCUPIED, int(thickness))

    def horizontal_wall(self, x0, x1, y, doors=()):
        half = self.door_width_m / 2.0
        cursor = x0
        for center in sorted(doors):
            left, right = center - half, center + half
            if left > cursor:
                self.segment(cursor, y, left, y)
            self.doors.append({"center_xy": [center, y], "orientation": "horizontal", "width_m": self.door_width_m})
            cursor = right
        if cursor < x1:
            self.segment(cursor, y, x1, y)

    def vertical_wall(self, x, y0, y1, doors=()):
        half = self.door_width_m / 2.0
        cursor = y0
        for center in sorted(doors):
            bottom, top = center - half, center + half
            if bottom > cursor:
                self.segment(x, cursor, x, bottom)
            self.doors.append({"center_xy": [x, center], "orientation": "vertical", "width_m": self.door_width_m})
            cursor = top
        if cursor < y1:
            self.segment(x, cursor, x, y1)

    def block(self, x0, y0, x1, y1, *, bottom_doors=(), top_doors=(), left_doors=(), right_doors=()):
        self.horizontal_wall(x0, x1, y0, bottom_doors)
        self.horizontal_wall(x0, x1, y1, top_doors)
        self.vertical_wall(x0, y0, y1, left_doors)
        self.vertical_wall(x1, y0, y1, right_doors)

    def room_grid(self, x0, y0, x1, y1, x_partitions, y_partitions):
        """Partition a block while leaving 2 m openings in every wall."""
        for index, x in enumerate(x_partitions):
            centers = [y0 + 12.0, y1 - 12.0]
            if index % 2:
                centers.append((y0 + y1) / 2.0)
            self.vertical_wall(x, y0, y1, centers)
        for index, y in enumerate(y_partitions):
            centers = [x0 + 12.0, x1 - 12.0]
            if index % 2:
                centers.append((x0 + x1) / 2.0)
            self.horizontal_wall(x0, x1, y, centers)

    def round_obstacle(self, x, y, radius_m):
        cv2.circle(self.image, self.point(x, y), round(radius_m / RESOLUTION), OCCUPIED, -1)

    def rectangle_obstacle(self, x0, y0, x1, y1, thickness_m=1.0):
        cv2.rectangle(self.image, self.point(x0, y1), self.point(x1, y0), OCCUPIED, round(thickness_m / RESOLUTION))

    def build(self):
        # Perimeter: the full canvas is 500 m x 400 m, with a closed 0.3 m edge.
        self.horizontal_wall(0, WIDTH_M, 0)
        self.horizontal_wall(0, WIDTH_M, HEIGHT_M)
        self.vertical_wall(0, 0, HEIGHT_M)
        self.vertical_wall(WIDTH_M, 0, HEIGHT_M)

        # Main circulation: a 10 m east-west avenue, a 10 m north-south avenue,
        # and a 100 m x 100 m open central plaza around their intersection.
        # Narrow parallel guide walls create explicit 4 m and 2 m passages.
        self.horizontal_wall(0, 190, 188, [45, 140])
        self.horizontal_wall(310, 500, 188, [360, 455])
        self.horizontal_wall(0, 190, 212, [75, 160])
        self.horizontal_wall(310, 500, 212, [350, 440])
        self.vertical_wall(188, 0, 140, [40, 100])
        self.vertical_wall(212, 0, 140, [70])
        self.vertical_wall(288, 260, 400, [300, 360])
        self.vertical_wall(312, 260, 400, [320, 370])

        # Four dense building blocks with alternating door positions.
        self.block(20, 230, 180, 380, bottom_doors=[55, 145], left_doors=[270, 345], right_doors=[285, 350])
        self.room_grid(20, 230, 180, 380, [60, 105, 150], [278, 326])

        self.block(320, 230, 480, 380, bottom_doors=[350, 445], left_doors=[270, 345], right_doors=[285, 350])
        self.room_grid(320, 230, 480, 380, [365, 410, 455], [278, 326])

        self.block(20, 20, 180, 150, top_doors=[55, 145], left_doors=[55, 120], right_doors=[70, 125])
        self.room_grid(20, 20, 180, 150, [60, 105, 150], [62, 104])

        self.block(320, 20, 480, 150, top_doors=[350, 445], left_doors=[55, 120], right_doors=[70, 125])
        self.room_grid(320, 20, 480, 150, [365, 410, 455], [62, 104])

        # Two open halls above and below the plaza, with multiple wide entries.
        self.block(190, 260, 310, 380, bottom_doors=[205, 250, 295], left_doors=[285, 350], right_doors=[285, 350])
        self.block(190, 20, 310, 140, top_doors=[205, 250, 295], left_doors=[45, 105], right_doors=[45, 105])

        # Warehouse aisles: long shelves leave 3 m and 5 m alternating passages.
        for y in (246, 264, 282, 300, 318, 336, 354):
            self.rectangle_obstacle(335, y, 465, y + 1.0, thickness_m=1.0)
        for x in (342, 390, 438):
            self.rectangle_obstacle(x, 240, x + 1.0, 365, thickness_m=1.0)

        # Large open zones and plazas include sparse columns/planters to create
        # broad and narrow path alternatives without sealing the area.
        for x, y, radius in (
            (70, 190, 3), (140, 190, 3), (360, 190, 3), (440, 190, 3),
            (210, 200, 2), (290, 200, 2), (250, 155, 4), (250, 245, 4),
            (205, 175, 1.5), (295, 175, 1.5), (205, 225, 1.5), (295, 225, 1.5),
        ):
            self.round_obstacle(x, y, radius)

        # A ring of small obstacles in the central plaza forms a wide open hall
        # with several approach directions.
        for angle in np.linspace(0, 2 * np.pi, 12, endpoint=False):
            self.round_obstacle(250 + 32 * np.cos(angle), 200 + 32 * np.sin(angle), 1.0)

        return self


def write_outputs(layout: Layout, output_dir: Path):
    output_dir.mkdir(parents=True, exist_ok=True)
    image_path = output_dir / "map.pgm"
    if not cv2.imwrite(str(image_path), layout.image):
        raise RuntimeError(f"failed to write {image_path}")
    yaml_text = (
        "image: map.pgm\n"
        "resolution: 0.05\n"
        "origin: [0.0, 0.0, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
        "mode: trinary\n"
    )
    (output_dir / "map.yaml").write_text(yaml_text)
    stats = {
        "width": layout.width,
        "height": layout.height,
        "resolution": RESOLUTION,
        "origin": [0.0, 0.0, 0.0],
        "extent_m": [WIDTH_M, HEIGHT_M],
        "canvas_area_m2": WIDTH_M * HEIGHT_M,
        "pixels": {str(value): int(np.count_nonzero(layout.image == value)) for value in (FREE, OCCUPIED, UNKNOWN)},
        "layout": {
            "wide_main_corridors_m": 10.0,
            "standard_corridors_m": 4.0,
            "narrow_passages_m": 2.0,
            "door_width_m": layout.door_width_m,
            "design_doors": len(layout.doors),
            "features": ["central 100x100 m plaza", "four room blocks", "two halls", "warehouse aisles", "sparse columns and planters"],
        },
    }
    (output_dir / "map_stats.json").write_text(json.dumps(stats, indent=2) + "\n")
    (output_dir / "SOURCE_LICENSE").write_text(
        "Generated synthetic Arena occupancy map.\n"
        "This map is an original layout generated by tools/generate_ultra_arena_map.py.\n"
        "No upstream mesh or raster asset is bundled.\n"
        "Map canvas: 500 m x 400 m = 200000 m2; resolution: 0.05 m/cell.\n"
    )
    (output_dir / "layout_design.json").write_text(json.dumps({"doors": layout.doors}, indent=2) + "\n")
    print(json.dumps(stats, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, help="map directory")
    args = parser.parse_args()
    write_outputs(Layout().build(), Path(args.output))


if __name__ == "__main__":
    main()
