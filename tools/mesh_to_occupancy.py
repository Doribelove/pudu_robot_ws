#!/usr/bin/env python3
"""Project selected Arena Gazebo meshes into ROS trinary occupancy maps.

This intentionally uses only numpy, OpenCV and the Python standard library.
The input meshes are model assets, so the result is an engineering projection
for 2-D navigation rather than a replacement for a hand-authored floor plan.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np


FREE = 254
UNKNOWN = 205
OCCUPIED = 0
COLLADA_NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}


def parse_obj(path: Path, campus: bool = False):
    """Read OBJ vertices and face polygons grouped by material."""
    vertices: list[tuple[float, float, float]] = []
    faces: dict[str, list[tuple[tuple[int, ...], float, float]]] = {}
    material = "__default__"
    with path.open("r", errors="ignore") as stream:
        for line in stream:
            if line.startswith("v "):
                fields = line.split()
                if len(fields) >= 4:
                    vertices.append((float(fields[1]), float(fields[2]), float(fields[3])))
            elif line.startswith("usemtl "):
                material = line.split(None, 1)[1].strip()
                faces.setdefault(material, [])
            elif line.startswith("f "):
                fields = line.split()[1:]
                if len(fields) < 3:
                    continue
                indices = []
                for field in fields:
                    # OBJ indices are 1-based and may be negative.
                    raw = field.split("/", 1)[0]
                    index = int(raw)
                    indices.append(index - 1 if index > 0 else len(vertices) + index)
                z = [vertices[i][2] for i in indices]
                faces.setdefault(material, []).append((tuple(indices), min(z), max(z)))
    return np.asarray(vertices, dtype=np.float64), faces


def parse_obj_parts(paths: Iterable[Path], free_parts: bool = False):
    """Read several OBJ files, assigning each whole part free or occupied."""
    all_vertices: list[np.ndarray] = []
    all_faces: list[tuple[tuple[int, ...], float, float, bool]] = []
    offset = 0
    for path in paths:
        vertices, faces = parse_obj(path)
        all_vertices.append(vertices)
        for polygons in faces.values():
            for indices, zmin, zmax in polygons:
                all_faces.append((tuple(i + offset for i in indices), zmin, zmax, free_parts))
        offset += len(vertices)
    return np.vstack(all_vertices), all_faces


def parse_collada(path: Path):
    """Return transformed triangle polygons from a Collada visual scene."""
    root = ET.parse(path).getroot()
    unit = root.find("c:asset/c:unit", COLLADA_NS)
    unit_scale = float(unit.attrib.get("meter", "1")) if unit is not None else 1.0

    geometries = {}
    for geometry in root.findall(".//c:library_geometries/c:geometry", COLLADA_NS):
        mesh = geometry.find("c:mesh", COLLADA_NS)
        if mesh is None:
            continue
        sources = {}
        for source in mesh.findall("c:source", COLLADA_NS):
            array = source.find("c:float_array", COLLADA_NS)
            if array is not None and array.text:
                sources[source.attrib["id"]] = np.asarray(
                    [float(x) for x in array.text.split()], dtype=np.float64
                )
        vertices = {}
        for vertex_set in mesh.findall("c:vertices", COLLADA_NS):
            for inp in vertex_set.findall("c:input", COLLADA_NS):
                if inp.attrib.get("semantic") == "POSITION":
                    vertices[vertex_set.attrib["id"]] = sources[inp.attrib["source"][1:]].reshape(-1, 3)
        geometries[geometry.attrib["id"]] = (mesh, sources, vertices)

    def local_matrix(node):
        matrix = node.find("c:matrix", COLLADA_NS)
        if matrix is not None and matrix.text:
            # Blender's Arena files serialize matrices row-major, with the
            # translation in elements 3, 7 and 11.
            return np.asarray([float(x) for x in matrix.text.split()], dtype=np.float64).reshape(4, 4)
        result = np.eye(4, dtype=np.float64)
        translate = node.find("c:translate", COLLADA_NS)
        if translate is not None and translate.text:
            result[:3, 3] = [float(x) for x in translate.text.split()]
        scale = node.find("c:scale", COLLADA_NS)
        if scale is not None and scale.text:
            sx, sy, sz = [float(x) for x in scale.text.split()]
            result[:3, :3] = np.diag([sx, sy, sz])
        return result

    triangles: list[tuple[np.ndarray, float, float]] = []
    scene = root.find(".//c:library_visual_scenes/c:visual_scene", COLLADA_NS)
    if scene is None:
        return triangles

    def visit(node, parent):
        world = parent @ local_matrix(node)
        instance = node.find("c:instance_geometry", COLLADA_NS)
        if instance is not None:
            item = geometries.get(instance.attrib["url"][1:])
            if item:
                mesh, sources, vertex_sets = item
                for primitive in list(mesh.findall("c:triangles", COLLADA_NS)) + list(mesh.findall("c:polylist", COLLADA_NS)):
                    inputs = primitive.findall("c:input", COLLADA_NS)
                    vertex_input = next((x for x in inputs if x.attrib.get("semantic") == "VERTEX"), None)
                    if vertex_input is None:
                        continue
                    offset = int(vertex_input.attrib.get("offset", 0))
                    stride = max(int(x.attrib.get("offset", 0)) for x in inputs) + 1
                    vertex_array = vertex_sets.get(vertex_input.attrib["source"][1:])
                    if vertex_array is None:
                        continue
                    values = [int(x) for x in (primitive.find("c:p", COLLADA_NS).text or "").split()]
                    if primitive.tag.endswith("polylist"):
                        counts = [int(x) for x in (primitive.find("c:vcount", COLLADA_NS).text or "").split()]
                        cursor = 0
                        polygons = []
                        for count in counts:
                            poly = [values[cursor + i * stride + offset] for i in range(count)]
                            cursor += count * stride
                            polygons.append(poly)
                        indices_groups = [p for p in polygons if len(p) >= 3]
                    else:
                        indices_groups = [
                            [values[i + j * stride + offset] for j in range(3)]
                            for i in range(0, len(values), 3 * stride)
                        ]
                    for indices in indices_groups:
                        points = vertex_array[np.asarray(indices, dtype=np.int64)] * unit_scale
                        homogeneous = np.c_[points, np.ones(len(points))]
                        transformed = homogeneous @ world.T
                        triangles.append((transformed[:, :3], float(transformed[:, 2].min()), float(transformed[:, 2].max())))
        for child in node.findall("c:node", COLLADA_NS):
            visit(child, world)

    for node in scene.findall("c:node", COLLADA_NS):
        visit(node, np.eye(4, dtype=np.float64))
    return triangles


def polygons_from_obj(vertices, faces, source_kind: str):
    free_names = {"floor", "floor.001", "floor.002", "floor.003", "floor.004", "floor.005", "grass__", "road"}
    free = []
    occupied = []
    for material, polygons in faces.items():
        is_free = source_kind == "campus" and material.lower() in free_names
        for indices, zmin, zmax in polygons:
            points = vertices[np.asarray(indices, dtype=np.int64)]
            if len(points) < 3:
                continue
            if is_free:
                free.append(points)
            elif zmax >= -0.05 and zmin <= 2.0:
                occupied.append(points)
    return free, occupied, vertices


def image_polygon(points: np.ndarray, min_x: float, max_y: float, resolution: float, width: int, height: int):
    xy = points[:, :2]
    px = np.rint((xy[:, 0] - min_x) / resolution).astype(np.int32)
    py = np.rint((max_y - xy[:, 1]) / resolution).astype(np.int32)
    return np.c_[px, py].reshape(-1, 1, 2)


def rasterize(free_polygons, occupied_polygons, vertices, output: Path, resolution: float, margin: float, hull_free: bool = False, inflate: float = 0.10):
    all_xy = vertices[:, :2] if len(vertices) else np.vstack([p[:, :2] for p in free_polygons + occupied_polygons])
    min_x, min_y = all_xy.min(axis=0) - margin
    max_x, max_y = all_xy.max(axis=0) + margin
    width = int(math.ceil((max_x - min_x) / resolution)) + 1
    height = int(math.ceil((max_y - min_y) / resolution)) + 1
    image = np.full((height, width), UNKNOWN, dtype=np.uint8)
    if hull_free and len(all_xy) >= 3:
        hull = cv2.convexHull(np.rint((all_xy - [min_x, min_y]) / resolution).astype(np.float32))
        hull[:, 0, 1] = np.rint((max_y - min_y) / resolution).astype(np.float32) - hull[:, 0, 1]
        cv2.fillPoly(image, [hull.astype(np.int32)], FREE)
    elif free_polygons:
        free_px = [image_polygon(p, min_x, max_y, resolution, width, height) for p in free_polygons]
        cv2.fillPoly(image, free_px, FREE)
    if occupied_polygons:
        occupied_px = [image_polygon(p, min_x, max_y, resolution, width, height) for p in occupied_polygons]
        occupied_mask = np.zeros_like(image)
        cv2.fillPoly(occupied_mask, occupied_px, 255)
        if inflate > 0:
            radius = max(1, int(math.ceil(inflate / resolution)))
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * radius + 1, 2 * radius + 1))
            occupied_mask = cv2.dilate(occupied_mask, kernel)
        image[occupied_mask > 0] = OCCUPIED
    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output), image):
        raise RuntimeError(f"failed to write {output}")
    stats = {
        "width": width,
        "height": height,
        "resolution": resolution,
        "origin": [round(float(min_x), 6), round(float(min_y), 6), 0.0],
        "extent_m": [round(width * resolution, 6), round(height * resolution, 6)],
        "canvas_area_m2": round(width * height * resolution * resolution, 3),
        "pixels": {str(value): int(np.count_nonzero(image == value)) for value in (FREE, OCCUPIED, UNKNOWN)},
    }
    return stats


def write_map_yaml(map_dir: Path, stats: dict):
    origin = ", ".join(f"{x:.6f}" for x in stats["origin"])
    text = (
        "image: map.pgm\n"
        f"resolution: {stats['resolution']:.2f}\n"
        f"origin: [{origin}]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
        "mode: trinary\n"
    )
    (map_dir / "map.yaml").write_text(text)
    (map_dir / "map_stats.json").write_text(json.dumps(stats, indent=2) + "\n")


def write_source_license(map_dir: Path, source: str):
    (map_dir / "SOURCE_LICENSE").write_text(
        "Converted from the open-source Arena simulation setup model.\n"
        "Source repository: https://github.com/voshch/arena-simulation-setup\n"
        "Source revision: 3f142b25d88ce962c803b57cf20f38985d376dea\n"
        f"Source asset: {source}\n"
        "Upstream package metadata declares BSD licensing. Preserve the "
        "upstream model metadata and author attribution when redistributing.\n"
        "This PGM/YAML is a generated 2-D projection at 0.05 m/cell; it is "
        "not an upstream raster map.\n"
    )


def convert(args):
    if args.kind == "campus":
        vertices, faces = parse_obj(Path(args.source), campus=True)
        free, occupied, vertices = polygons_from_obj(vertices, faces, "campus")
        return free, occupied, vertices, False
    if args.kind == "airport":
        source = Path(args.source)
        floors = sorted(source.glob("floor_*.obj"))
        walls = sorted(source.glob("wall_*.obj"))
        floor_vertices, floor_faces = parse_obj_parts(floors, free_parts=True)
        wall_vertices, wall_faces = parse_obj_parts(walls, free_parts=False)
        vertices = np.vstack([floor_vertices, wall_vertices])
        offset = len(floor_vertices)
        free = []
        occupied = []
        for indices, zmin, zmax, _ in floor_faces:
            # Each floor OBJ contains a thin solid: keep its upper face and
            # drop the reverse-facing underside, which would cancel adjacent
            # contours when OpenCV rasterizes the whole mesh at once.
            if zmax >= -0.05:
                free.append(floor_vertices[np.asarray(indices, dtype=np.int64)])
        for indices, zmin, zmax, _ in wall_faces:
            points = wall_vertices[np.asarray(indices, dtype=np.int64)]
            if zmax >= -0.05 and zmin <= 2.0:
                occupied.append(points)
        return free, occupied, vertices, False
    if args.kind == "dae":
        triangles = parse_collada(Path(args.source))
        vertices = np.vstack([p for p, _, _ in triangles]) if triangles else np.empty((0, 3))
        occupied = [p for p, zmin, zmax in triangles if zmax >= -0.05 and zmin <= 2.0]
        return [], occupied, vertices, True
    raise ValueError(args.kind)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--kind", choices=("campus", "airport", "dae"), required=True)
    parser.add_argument("--source", required=True, help="OBJ file, airport mesh directory, or DAE file")
    parser.add_argument("--output", required=True, help="map directory containing map.pgm/map.yaml")
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--margin", type=float, default=0.25)
    parser.add_argument("--inflate", type=float, default=0.10)
    args = parser.parse_args()
    if args.resolution != 0.05:
        raise SystemExit("This Arena asset set is standardized at --resolution 0.05")
    free, occupied, vertices, hull_free = convert(args)
    output_dir = Path(args.output)
    stats = rasterize(free, occupied, vertices, output_dir / "map.pgm", args.resolution, args.margin, hull_free, args.inflate)
    write_map_yaml(output_dir, stats)
    write_source_license(output_dir, args.source)
    print(json.dumps(stats, indent=2))


if __name__ == "__main__":
    main()
