#!/usr/bin/env python3
"""Generate eight connected synthetic Arena maps in four area bands."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


RESOLUTION = 0.05
FREE = 254
UNKNOWN = 205
OCCUPIED = 0
WALL_M = 0.25
DOOR_M = 2.0

MAPS = {
    "arena_small_080x100_2d_005": (80.0, 100.0, "small"),
    "arena_small_100x100_2d_005": (100.0, 100.0, "small"),
    "arena_medium_160x200_2d_005": (160.0, 200.0, "medium"),
    "arena_medium_200x200_2d_005": (200.0, 200.0, "medium"),
    "arena_large_250x300_2d_005": (250.0, 300.0, "large"),
    "arena_large_300x500_2d_005": (300.0, 500.0, "large"),
    "arena_xlarge_500x500_2d_005": (500.0, 500.0, "xlarge"),
    "arena_xlarge_600x400_2d_005": (600.0, 400.0, "xlarge"),
}


class Plan:
    def __init__(self, width_m: float, height_m: float, variant: str):
        self.width_m = width_m
        self.height_m = height_m
        self.width = round(width_m / RESOLUTION)
        self.height = round(height_m / RESOLUTION)
        self.variant = variant
        self.image = np.full((self.height, self.width), FREE, dtype=np.uint8)
        self.wall_cells = max(4, round(WALL_M / RESOLUTION))
        self.design_doors = []

    def point(self, x: float, y: float):
        return round(x / RESOLUTION), round(self.height_m / RESOLUTION - y / RESOLUTION)

    def segment(self, x0, y0, x1, y1, thickness=None):
        cv2.line(self.image, self.point(x0, y0), self.point(x1, y1), OCCUPIED, thickness or self.wall_cells)

    def hwall(self, x0, x1, y, doors=()):
        half = DOOR_M / 2.0
        cursor = x0
        for center in sorted(doors):
            left, right = center - half, center + half
            if left > cursor:
                self.segment(cursor, y, left, y)
            self.design_doors.append({"center_xy": [round(center, 3), round(y, 3)], "width_m": DOOR_M, "orientation": "horizontal"})
            cursor = right
        if cursor < x1:
            self.segment(cursor, y, x1, y)

    def vwall(self, x, y0, y1, doors=()):
        half = DOOR_M / 2.0
        cursor = y0
        for center in sorted(doors):
            bottom, top = center - half, center + half
            if bottom > cursor:
                self.segment(x, cursor, x, bottom)
            self.design_doors.append({"center_xy": [round(x, 3), round(center, 3)], "width_m": DOOR_M, "orientation": "vertical"})
            cursor = top
        if cursor < y1:
            self.segment(x, cursor, x, y1)

    def shelf_h(self, x0, x1, y, gaps):
        self.hwall(x0, x1, y, gaps)

    def shelf_v(self, x, y0, y1, gaps):
        self.vwall(x, y0, y1, gaps)

    def block(self, x0, y0, x1, y1, warehouse=False):
        # Every block has at least two external 2 m entries on each side.
        self.hwall(x0, x1, y0, [x0 + 0.30 * (x1 - x0), x0 + 0.70 * (x1 - x0)])
        self.hwall(x0, x1, y1, [x0 + 0.30 * (x1 - x0), x0 + 0.70 * (x1 - x0)])
        self.vwall(x0, y0, y1, [y0 + 0.30 * (y1 - y0), y0 + 0.70 * (y1 - y0)])
        self.vwall(x1, y0, y1, [y0 + 0.30 * (y1 - y0), y0 + 0.70 * (y1 - y0)])
        if warehouse:
            # Shelf gaps are aligned in both axes, so every aisle can reach
            # both the block doors and the main corridor.
            for y in np.linspace(y0 + 0.16 * (y1 - y0), y1 - 0.16 * (y1 - y0), 5):
                self.shelf_h(x0 + 4, x1 - 4, float(y), [x0 + 0.50 * (x1 - x0)])
            for x in np.linspace(x0 + 0.16 * (x1 - x0), x1 - 0.16 * (x1 - x0), 4):
                self.shelf_v(float(x), y0 + 4, y1 - 4, [y0 + 0.50 * (y1 - y0)])
            return
        # Alternating door locations make room-to-room paths less uniform and
        # create both direct and turning approaches.
        for index, x in enumerate(np.linspace(x0 + 0.25 * (x1 - x0), x1 - 0.25 * (x1 - x0), 2)):
            doors = [y0 + 0.25 * (y1 - y0), y1 - 0.25 * (y1 - y0)]
            if index % 2:
                doors.append(y0 + 0.50 * (y1 - y0))
            self.vwall(float(x), y0, y1, doors)
        for index, y in enumerate(np.linspace(y0 + 0.33 * (y1 - y0), y1 - 0.33 * (y1 - y0), 2)):
            doors = [x0 + 0.25 * (x1 - x0), x1 - 0.25 * (x1 - x0)]
            if index % 2:
                doors.append(x0 + 0.50 * (x1 - x0))
            self.hwall(x0, x1, float(y), doors)

    def circle(self, x, y, radius):
        cv2.circle(self.image, self.point(x, y), max(1, round(radius / RESOLUTION)), OCCUPIED, -1)

    def build(self):
        m = max(2.0, min(self.width_m, self.height_m) * 0.04)
        cx0, cx1 = self.width_m * 0.35, self.width_m * 0.65
        cy0, cy1 = self.height_m * 0.32, self.height_m * 0.68

        # Perimeter walls keep the 2-D canvas bounded without adding unknown
        # pixels. The complete canvas is therefore the advertised area.
        self.hwall(0, self.width_m, 0)
        self.hwall(0, self.width_m, self.height_m)
        self.vwall(0, 0, self.height_m)
        self.vwall(self.width_m, 0, self.height_m)

        # Four room blocks surround a central open plaza and intersecting main
        # corridors. The gap between blocks is intentionally wide (> 3 m).
        block_w = max(0.22 * self.width_m, 18.0)
        # Keep the blocks compact enough that even the 100 m maps have
        # explicit wide-hall envelopes between the blocks and plaza.
        block_h = max(0.12 * self.height_m, 12.0)
        left_x = (m, min(self.width_m * 0.31, m + block_w))
        right_x = (max(self.width_m * 0.69, self.width_m - m - block_w), self.width_m - m)
        bottom_y = (m, min(self.height_m * 0.27, m + block_h))
        top_y = (max(self.height_m * 0.73, self.height_m - m - block_h), self.height_m - m)
        self.block(left_x[0], top_y[0], left_x[1], top_y[1])
        self.block(right_x[0], top_y[0], right_x[1], top_y[1], warehouse=True)
        self.block(left_x[0], bottom_y[0], left_x[1], bottom_y[1])
        self.block(right_x[0], bottom_y[0], right_x[1], bottom_y[1])

        # Optional hall envelopes above/below the plaza. Their broad entries
        # are 2 m doors, while the surrounding open circulation remains > 3 m.
        hall_x0, hall_x1 = self.width_m * 0.39, self.width_m * 0.61
        hall_top_y0, hall_top_y1 = cy1 + 0.03 * self.height_m, top_y[0] - 0.02 * self.height_m
        hall_bot_y0, hall_bot_y1 = bottom_y[1] + 0.02 * self.height_m, cy0 - 0.03 * self.height_m
        if hall_top_y1 - hall_top_y0 > 8:
            self.hwall(hall_x0, hall_x1, hall_top_y0, [hall_x0 + 0.3 * (hall_x1 - hall_x0), hall_x0 + 0.7 * (hall_x1 - hall_x0)])
            self.hwall(hall_x0, hall_x1, hall_top_y1, [hall_x0 + 0.5 * (hall_x1 - hall_x0)])
            self.vwall(hall_x0, hall_top_y0, hall_top_y1, [hall_top_y0 + 0.5 * (hall_top_y1 - hall_top_y0)])
            self.vwall(hall_x1, hall_top_y0, hall_top_y1, [hall_top_y0 + 0.5 * (hall_top_y1 - hall_top_y0)])
        if hall_bot_y1 - hall_bot_y0 > 8:
            self.hwall(hall_x0, hall_x1, hall_bot_y0, [hall_x0 + 0.5 * (hall_x1 - hall_x0)])
            self.hwall(hall_x0, hall_x1, hall_bot_y1, [hall_x0 + 0.3 * (hall_x1 - hall_x0), hall_x0 + 0.7 * (hall_x1 - hall_x0)])
            self.vwall(hall_x0, hall_bot_y0, hall_bot_y1, [hall_bot_y0 + 0.5 * (hall_bot_y1 - hall_bot_y0)])
            self.vwall(hall_x1, hall_bot_y0, hall_bot_y1, [hall_bot_y0 + 0.5 * (hall_bot_y1 - hall_bot_y0)])

        # Wide east-west and north-south circulation is represented by the
        # open gaps around the blocks. A pair of long guide walls leaves 4 m
        # side passages and a 10 m central plaza crossing.
        corridor_half = max(2.0, min(self.width_m, self.height_m) * 0.025)
        self.hwall(m, self.width_m - m, cy0 - corridor_half, [cx0, cx1])
        self.hwall(m, self.width_m - m, cy1 + corridor_half, [cx0, cx1])
        self.vwall(cx0 - corridor_half, m, self.height_m - m, [cy0, cy1])
        self.vwall(cx1 + corridor_half, m, self.height_m - m, [cy0, cy1])

        # Central plaza columns and a circular feature create broad and narrow
        # route alternatives while keeping the plaza itself open.
        for angle in np.linspace(0, 2 * np.pi, 12, endpoint=False):
            self.circle(self.width_m * 0.5 + min(self.width_m, self.height_m) * 0.10 * np.cos(angle), self.height_m * 0.5 + min(self.width_m, self.height_m) * 0.10 * np.sin(angle), 0.8)
        self.circle(self.width_m * 0.5, self.height_m * 0.5, 2.5)
        return self


def write_map(name: str, width_m: float, height_m: float, variant: str, output: Path):
    plan = Plan(width_m, height_m, variant).build()
    output.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output / "map.pgm"), plan.image):
        raise RuntimeError(f"failed to write {output / 'map.pgm'}")
    (output / "map.yaml").write_text(
        "image: map.pgm\n"
        "resolution: 0.05\n"
        "origin: [0.0, 0.0, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
        "mode: trinary\n"
    )
    stats = {
        "width": plan.width,
        "height": plan.height,
        "resolution": RESOLUTION,
        "origin": [0.0, 0.0, 0.0],
        "extent_m": [width_m, height_m],
        "canvas_area_m2": width_m * height_m,
        "pixels": {str(value): int(np.count_nonzero(plan.image == value)) for value in (FREE, OCCUPIED, UNKNOWN)},
        "layout": {
            "variant": variant,
            "design_doors": len(plan.design_doors),
            "door_width_m": DOOR_M,
            "narrow_passage_min_m": 2.0,
            "wide_corridor_min_m": 4.0,
            "features": ["central open plaza", "east-west main corridor", "north-south main corridor", "wide halls", "2 m doors", "office/teaching room grids", "warehouse shelf aisles"],
        },
    }
    (output / "map_stats.json").write_text(json.dumps(stats, indent=2) + "\n")
    (output / "layout_design.json").write_text(json.dumps({"doors": plan.design_doors}, indent=2) + "\n")
    (output / "SOURCE_LICENSE").write_text(
        "Generated synthetic Arena occupancy map.\n"
        "Original layout generated by tools/generate_arena_scale_maps.py.\n"
        "No upstream mesh or raster asset is bundled.\n"
        f"Map canvas: {width_m:g} m x {height_m:g} m = {width_m * height_m:g} m2; resolution: 0.05 m/cell.\n"
    )
    scenario_dir = output.parent / "scenarios"
    scenario_dir.mkdir(parents=True, exist_ok=True)
    scenario_offset = min(width_m, height_m) * 0.08
    (scenario_dir / "default.json").write_text(
        json.dumps(
            {
                "robots": [
                    {
                        "start": [width_m * 0.5 - scenario_offset, height_m * 0.5, 0.0],
                        "goal": [width_m * 0.5 + scenario_offset, height_m * 0.5, 0.0],
                    }
                ],
                "obstacles": {"static": [], "dynamic": [], "interactive": []},
            },
            indent=2,
        )
        + "\n"
    )
    return stats


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", required=True)
    parser.add_argument("--name", choices=sorted(MAPS))
    args = parser.parse_args()
    root = Path(args.output_root)
    names = [args.name] if args.name else list(MAPS)
    for name in names:
        stats = write_map(name, *MAPS[name], root / name / "map")
        print(json.dumps({"name": name, **stats}, indent=2))


if __name__ == "__main__":
    main()
