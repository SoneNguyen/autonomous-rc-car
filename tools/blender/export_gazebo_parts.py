"""Export the single Truck mesh in chassis.blend as Gazebo-ready STL parts.

Run from the repository root with Blender (the script uses bpy):

    blender --background assets/source/chassis.blend --python tools/blender/export_gazebo_parts.py

The source asset is one mesh object with many disconnected islands. The four
largest equal islands are tyre/rim seeds. This script also attaches the small
disconnected hub and rim details fully inside each seed-wheel envelope, while
leaving the remaining decorative islands in the chassis mesh.

Output coordinates use Gazebo's vehicle convention: +X forward, +Y left,
+Z up.  Wheel meshes are centered at their hub, while chassis.stl is centered
at the midpoint of all four hubs.  Exact poses and dimensions are written to
meshes/mesh_manifest.json for model.sdf and any downstream tooling.
"""

from __future__ import annotations

from collections import defaultdict
import json
import math
from pathlib import Path
import struct

import bpy


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
OUTPUT_DIRECTORY = REPOSITORY_ROOT / "simulation" / "models" / "car" / "meshes"
WHEEL_PART_NAMES = ("wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr")


class UnionFind:
    def __init__(self, size: int) -> None:
        self.parent = list(range(size))

    def find(self, value: int) -> int:
        while self.parent[value] != value:
            self.parent[value] = self.parent[self.parent[value]]
            value = self.parent[value]
        return value

    def union(self, first: int, second: int) -> None:
        first_root = self.find(first)
        second_root = self.find(second)
        if first_root != second_root:
            self.parent[second_root] = first_root


def vector_tuple(value) -> tuple[float, float, float]:
    return (float(value.x), float(value.y), float(value.z))


def bounds(points: list[tuple[float, float, float]]) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    return (
        tuple(min(point[axis] for point in points) for axis in range(3)),
        tuple(max(point[axis] for point in points) for axis in range(3)),
    )


def center_of_bounds(part_bounds: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    minimum, maximum = part_bounds
    return tuple((minimum[axis] + maximum[axis]) / 2.0 for axis in range(3))


def subtract(first: tuple[float, float, float], second: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(first[axis] - second[axis] for axis in range(3))


def blender_to_gazebo(point: tuple[float, float, float]) -> tuple[float, float, float]:
    """Rotate Blender (+Y forward, +X right, +Z up) to Gazebo vehicle axes."""
    return (point[1], -point[0], point[2])


def triangle_normal(
    first: tuple[float, float, float],
    second: tuple[float, float, float],
    third: tuple[float, float, float],
) -> tuple[float, float, float]:
    first_edge = tuple(second[axis] - first[axis] for axis in range(3))
    second_edge = tuple(third[axis] - first[axis] for axis in range(3))
    normal = (
        first_edge[1] * second_edge[2] - first_edge[2] * second_edge[1],
        first_edge[2] * second_edge[0] - first_edge[0] * second_edge[2],
        first_edge[0] * second_edge[1] - first_edge[1] * second_edge[0],
    )
    magnitude = math.sqrt(sum(component * component for component in normal))
    if magnitude == 0.0:
        return (0.0, 0.0, 0.0)
    return tuple(component / magnitude for component in normal)


def write_binary_stl(
    destination: Path,
    name: str,
    triangles: list[tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]],
) -> None:
    """Write a compact binary STL without depending on Blender export settings."""
    header = f"Gazebo-ready part exported from chassis.blend: {name}".encode("ascii")[:80]
    with destination.open("wb") as stream:
        stream.write(header.ljust(80, b"\0"))
        stream.write(struct.pack("<I", len(triangles)))
        for first, second, third in triangles:
            normal = triangle_normal(first, second, third)
            stream.write(struct.pack("<12fH", *(normal + first + second + third), 0))


def transformed_triangles(
    source_triangles: list[tuple[int, int, int]],
    world_vertices: list[tuple[float, float, float]],
    origin: tuple[float, float, float],
) -> list[tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]]:
    return [
        tuple(blender_to_gazebo(subtract(world_vertices[index], origin)) for index in triangle)
        for triangle in source_triangles
    ]


def describe_part(
    filename: str,
    triangles: list[tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]],
) -> dict[str, object]:
    points = [point for triangle in triangles for point in triangle]
    minimum, maximum = bounds(points)
    return {
        "file": filename,
        "triangles": len(triangles),
        "bounds": {"min": minimum, "max": maximum},
        "size": tuple(maximum[axis] - minimum[axis] for axis in range(3)),
    }


def main() -> None:
    mesh_objects = [object for object in bpy.context.scene.objects if object.type == "MESH"]
    if len(mesh_objects) != 1:
        raise RuntimeError(f"Expected exactly one mesh object in the source blend; found {len(mesh_objects)}.")

    source_object = mesh_objects[0]
    source_mesh = source_object.data
    source_mesh.calc_loop_triangles()

    connectivity = UnionFind(len(source_mesh.vertices))
    for polygon in source_mesh.polygons:
        polygon_vertices = list(polygon.vertices)
        for vertex in polygon_vertices[1:]:
            connectivity.union(polygon_vertices[0], vertex)

    component_polygons: dict[int, list[int]] = defaultdict(list)
    for polygon in source_mesh.polygons:
        component_polygons[connectivity.find(polygon.vertices[0])].append(polygon.index)

    world_vertices = [vector_tuple(source_object.matrix_world @ vertex.co) for vertex in source_mesh.vertices]
    components = []
    for root, polygon_indices in component_polygons.items():
        vertex_indices = {vertex for polygon_index in polygon_indices for vertex in source_mesh.polygons[polygon_index].vertices}
        component_bounds = bounds([world_vertices[vertex] for vertex in vertex_indices])
        material_names = {
            source_mesh.materials[source_mesh.polygons[polygon_index].material_index].name
            for polygon_index in polygon_indices
            if source_mesh.polygons[polygon_index].material_index < len(source_mesh.materials)
        }
        components.append(
            {
                "root": root,
                "polygons": polygon_indices,
                "vertex_indices": vertex_indices,
                "bounds": component_bounds,
                "hub": center_of_bounds(component_bounds),
                "materials": material_names,
            }
        )

    components.sort(key=lambda component: len(component["polygons"]), reverse=True)
    if len(components) < 4:
        raise RuntimeError("The source mesh does not contain four disconnected wheel candidates.")

    wheels = components[:4]
    wheel_face_counts = {len(wheel["polygons"]) for wheel in wheels}
    if len(wheel_face_counts) != 1:
        raise RuntimeError(f"The four largest islands are not equivalent wheel candidates: {wheel_face_counts}.")

    # The source uses Blender axes: +Y points forward and +X points right.
    # Front wheels have the highest Y coordinate, and left wheels the lowest X.
    front_wheels = sorted(wheels, key=lambda wheel: wheel["hub"][1], reverse=True)[:2]
    rear_wheels = sorted(wheels, key=lambda wheel: wheel["hub"][1])[:2]
    if min(wheel["hub"][1] for wheel in front_wheels) <= max(wheel["hub"][1] for wheel in rear_wheels):
        raise RuntimeError("Could not separate front and rear wheel centers.")

    def left_and_right(pair):
        ordered = sorted(pair, key=lambda wheel: wheel["hub"][0])
        return ordered[0], ordered[1]

    front_left, front_right = left_and_right(front_wheels)
    rear_left, rear_right = left_and_right(rear_wheels)
    named_wheels = {
        "wheel_fl": front_left,
        "wheel_fr": front_right,
        "wheel_rl": rear_left,
        "wheel_rr": rear_right,
    }
    if set(named_wheels) != set(WHEEL_PART_NAMES):
        raise RuntimeError("Wheel naming did not complete.")

    root_origin = tuple(sum(wheel["hub"][axis] for wheel in wheels) / len(wheels) for axis in range(3))
    # Wheel rims, caps, and inner discs are disconnected from the tyre mesh.
    # Include islands wholly inside their seed-wheel bounds, except for the two
    # one-face Body fragments which are fender/body geometry despite coinciding
    # with the front tyre envelopes. This avoids a nearest-centre heuristic
    # that would incorrectly move nearby body trim with a wheel.
    containment_epsilon = 0.02

    def is_inside_seed(component, seed) -> bool:
        minimum, maximum = component["bounds"]
        seed_minimum, seed_maximum = seed["bounds"]
        return all(
            minimum[axis] >= seed_minimum[axis] - containment_epsilon
            and maximum[axis] <= seed_maximum[axis] + containment_epsilon
            for axis in range(3)
        )

    wheel_component_roots: dict[str, set[int]] = {}
    for name, seed in named_wheels.items():
        wheel_component_roots[name] = {
            component["root"]
            for component in components
            if is_inside_seed(component, seed) and "Body" not in component["materials"]
        }
        wheel_component_roots[name].add(seed["root"])

    polygon_to_component = {
        polygon_index: root
        for root, polygon_indices in component_polygons.items()
        for polygon_index in polygon_indices
    }
    triangles_by_component: dict[int, list[tuple[int, int, int]]] = defaultdict(list)
    for triangle in source_mesh.loop_triangles:
        component_root = polygon_to_component[triangle.polygon_index]
        triangles_by_component[component_root].append(tuple(triangle.vertices))

    wheel_roots = set().union(*wheel_component_roots.values())
    chassis_source_triangles = [
        tuple(triangle.vertices)
        for triangle in source_mesh.loop_triangles
        if polygon_to_component[triangle.polygon_index] not in wheel_roots
    ]

    OUTPUT_DIRECTORY.mkdir(parents=True, exist_ok=True)
    chassis_triangles = transformed_triangles(chassis_source_triangles, world_vertices, root_origin)
    write_binary_stl(OUTPUT_DIRECTORY / "chassis.stl", "chassis", chassis_triangles)

    manifest_parts: dict[str, object] = {
        "chassis": describe_part("chassis.stl", chassis_triangles),
    }
    for name in WHEEL_PART_NAMES:
        wheel = named_wheels[name]
        source_triangles = [
            triangle
            for component_root in wheel_component_roots[name]
            for triangle in triangles_by_component[component_root]
        ]
        part_triangles = transformed_triangles(source_triangles, world_vertices, wheel["hub"])
        write_binary_stl(OUTPUT_DIRECTORY / f"{name}.stl", name, part_triangles)
        part_description = describe_part(f"{name}.stl", part_triangles)
        part_description["source_component_count"] = len(wheel_component_roots[name])
        gazebo_pose = blender_to_gazebo(subtract(wheel["hub"], root_origin))
        part_description["hub_pose_relative_to_chassis"] = gazebo_pose
        part_description["wheel_radius"] = part_description["size"][2] / 2.0
        part_description["wheel_width"] = part_description["size"][1]
        manifest_parts[name] = part_description

    manifest = {
        "source": Path(bpy.data.filepath).name,
        "source_mesh_object": source_object.name,
        "coordinate_system": {
            "source": "Blender: +Y forward, +X right, +Z up",
            "output": "Gazebo vehicle frame: +X forward, +Y left, +Z up",
            "transform": "gazebo = (blender_y, -blender_x, blender_z)",
            "units": "Source units retained; STL itself does not encode units.",
        },
        "chassis_origin_in_source_frame": root_origin,
        "chassis_origin_in_gazebo_frame": (0.0, 0.0, 0.0),
        "source_component_count": len(components),
        "parts": manifest_parts,
    }
    with (OUTPUT_DIRECTORY / "mesh_manifest.json").open("w", encoding="utf-8") as stream:
        json.dump(manifest, stream, indent=2)
        stream.write("\n")

    total_triangles = len(chassis_triangles) + sum(manifest_parts[name]["triangles"] for name in WHEEL_PART_NAMES)
    print(f"Exported {total_triangles} triangles from {source_object.name} to {OUTPUT_DIRECTORY}")
    for name in ("chassis",) + WHEEL_PART_NAMES:
        print(f"  {name}: {manifest_parts[name]['triangles']} triangles")


if __name__ == "__main__":
    main()
