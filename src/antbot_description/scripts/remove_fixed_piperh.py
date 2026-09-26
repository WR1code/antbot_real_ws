#!/usr/bin/env python3
"""Remove the CAD Piper-H assembly while preserving the production chassis.

The source mesh is a binary STL exported from the complete SolidWorks
assembly.  Parts are disconnected shells, so they can be classified by their
component centroid in the original millimetre-based CAD coordinate system.
"""

import argparse
from pathlib import Path
import struct


STL_HEADER_SIZE = 84
FACET_SIZE = 50
EXPECTED_INPUT_FACETS = 171_491
EXPECTED_REMOVED_FACETS = 41_378


class DisjointSet:
    def __init__(self):
        self.parents = []
        self.ranks = []

    def add(self):
        item = len(self.parents)
        self.parents.append(item)
        self.ranks.append(0)
        return item

    def find(self, item):
        while self.parents[item] != item:
            self.parents[item] = self.parents[self.parents[item]]
            item = self.parents[item]
        return item

    def union(self, first, second):
        first = self.find(first)
        second = self.find(second)
        if first == second:
            return
        if self.ranks[first] < self.ranks[second]:
            first, second = second, first
        self.parents[second] = first
        if self.ranks[first] == self.ranks[second]:
            self.ranks[first] += 1


def _load_facets(path):
    data = path.read_bytes()
    if len(data) < STL_HEADER_SIZE:
        raise ValueError(f"{path} is not a binary STL")
    count = struct.unpack_from("<I", data, 80)[0]
    expected_size = STL_HEADER_SIZE + FACET_SIZE * count
    if len(data) != expected_size:
        raise ValueError(
            f"{path} has {len(data)} bytes; expected {expected_size} for "
            f"{count} binary STL facets"
        )
    facets = [
        data[STL_HEADER_SIZE + FACET_SIZE * index:
             STL_HEADER_SIZE + FACET_SIZE * (index + 1)]
        for index in range(count)
    ]
    return data[:80], facets


def _component_roots(facets):
    sets = DisjointSet()
    vertices = {}
    facet_vertices = []
    for facet in facets:
        values = struct.unpack("<12fH", facet)
        ids = []
        for vertex_index in range(3):
            # The STEP-to-STL conversion can differ below a tenth of a micron
            # at a shared edge. Four decimal mm quantisation reconnects only
            # coincident vertices, not nearby mechanical parts.
            vertex = tuple(
                round(values[3 + 3 * vertex_index + axis], 4)
                for axis in range(3)
            )
            if vertex not in vertices:
                vertices[vertex] = sets.add()
            ids.append(vertices[vertex])
        sets.union(ids[0], ids[1])
        sets.union(ids[0], ids[2])
        facet_vertices.append(ids[0])
    return sets, facet_vertices


def remove_fixed_piperh(source, destination):
    header, facets = _load_facets(source)
    if len(facets) != EXPECTED_INPUT_FACETS:
        raise ValueError(
            f"refusing to classify an unexpected CAD revision: got "
            f"{len(facets)} facets, expected {EXPECTED_INPUT_FACETS}"
        )

    sets, facet_vertices = _component_roots(facets)
    bounds = {}
    for facet, vertex_id in zip(facets, facet_vertices):
        root = sets.find(vertex_id)
        low, high = bounds.setdefault(root, ([float("inf")] * 3, [float("-inf")] * 3))
        values = struct.unpack("<12fH", facet)
        for vertex_index in range(3):
            for axis in range(3):
                value = values[3 + 3 * vertex_index + axis]
                low[axis] = min(low[axis], value)
                high[axis] = max(high[axis], value)

    # Piper-H occupies the negative-Y side of the source CAD. The X cutoff
    # excludes wheels and lower chassis hardware that share that half-space.
    removed_roots = {
        root
        for root, (low, high) in bounds.items()
        if (low[0] + high[0]) * 0.5 > 200.0
        and (low[1] + high[1]) * 0.5 < -115.0
    }
    keep = [
        facet
        for facet, vertex_id in zip(facets, facet_vertices)
        if sets.find(vertex_id) not in removed_roots
    ]
    removed_count = len(facets) - len(keep)
    if removed_count != EXPECTED_REMOVED_FACETS:
        raise RuntimeError(
            f"component classification removed {removed_count} facets; "
            f"expected {EXPECTED_REMOVED_FACETS}"
        )

    output_header = header[:80].ljust(80, b" ")
    destination.write_bytes(
        output_header + struct.pack("<I", len(keep)) + b"".join(keep)
    )
    return len(keep), removed_count


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()
    kept, removed = remove_fixed_piperh(args.source, args.destination)
    print(f"wrote {args.destination}: kept {kept}, removed {removed} facets")


if __name__ == "__main__":
    main()
