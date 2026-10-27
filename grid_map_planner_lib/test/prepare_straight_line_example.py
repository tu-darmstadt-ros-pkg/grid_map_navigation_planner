#!/usr/bin/env python3
"""Prepare an external fixture config without importing/executing the supplied scripts."""
import argparse
import ast
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("example_dir", type=Path, help="Extracted straight-line-repro directory")
    parser.add_argument("output", type=Path, help="Private output config, outside the source repository")
    args = parser.parse_args()
    example = args.example_dir.resolve()
    tree = ast.parse((example / "repro_straight_line.py").read_text())
    names = {"D", "A0", "A8", "P", "Q1", "B21"}
    points = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1:
            target = node.targets[0]
            if isinstance(target, ast.Name) and target.id in names:
                points[target.id] = ast.literal_eval(node.value)
    if points.keys() != names:
        raise ValueError("Missing reproduction coordinates")
    maps = list((example / "maps").glob("*/*_nav_map.yaml"))
    if len(maps) != 1:
        raise ValueError("Expected exactly one navigation map")
    def route(start, goal):
        return [points[start], points[goal]]
    config = {
        "map": str(maps[0]),
        "zones": str(maps[0].parent / "zones.yaml"),
        "triggers": [route("A8", "P"), route("P", "Q1")],
        "routes": [route("D", "A0"), route("B21", "D")],
    }
    args.output.write_text(json.dumps(config, indent=2) + "\n")
    print(args.output)


if __name__ == "__main__":
    main()
