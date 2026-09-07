"""CLI for associating existing 2D map, 3D cloud, and waypoint files."""

import argparse
import sys

from .pcd_io import create_map_bundle


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", required=True)
    parser.add_argument("--map-yaml")
    parser.add_argument("--cloud")
    parser.add_argument("--metadata")
    parser.add_argument("--waypoints")
    parser.add_argument("--output", required=True)
    parser.add_argument("--notes", default="")
    parsed = parser.parse_args(sys.argv[1:] if args is None else args)
    if not any((parsed.map_yaml, parsed.cloud, parsed.metadata, parsed.waypoints)):
        parser.error("at least one input artifact is required")
    try:
        manifest = create_map_bundle(
            parsed.output,
            parsed.name,
            map_yaml=parsed.map_yaml,
            cloud=parsed.cloud,
            metadata=parsed.metadata,
            waypoints=parsed.waypoints,
            notes=parsed.notes,
        )
    except Exception as error:
        parser.exit(1, f"create_map_bundle failed: {error}\n")
    print(f"map bundle created: {parsed.output}")
    print(f"manifest: {parsed.output}/manifest.yaml")
    for key in (
        "occupancy_map_yaml", "pointcloud_file",
        "pointcloud_metadata_file", "waypoint_file",
    ):
        print(f"{key}: {manifest[key] or '(not present)'}")
