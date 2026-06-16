#!/usr/bin/env python3
"""Replace the Stretch Assist ArUco marker geoms with single textured boxes.

Each marker becomes one thin textured box (material ``stretch_assist_aruco_<id>
_mat``, defined in scene.xml's <asset>) instead of a white base plus ~16 black
tiles. The textured plane renders crisply at small sizes, so the markers can
shrink to ~5 cm and still decode. Run tools/gen_aruco_textures.py once first to
create the PNG textures.

Handles either the old tile form or a previously generated textured box, so it
is safe to re-run to change the size.

Marker size can be set per object so each one reads as a label that fits its
top face (e.g. the narrow glass gets a smaller marker than the boxes).

Usage:
    python tools/regen_aruco_textured.py --base-size 0.05
    python tools/regen_aruco_textured.py --base-size 0.05 --size glass=0.03
    python tools/regen_aruco_textured.py --base-size 0.05 --dry-run
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path

SCENE = Path(__file__).resolve().parent.parent / "stretch_mujoco" / "models" / "scene.xml"

# Matches every geom belonging to a marker: the old white_base / black_r_c tiles
# and the new single "marker" box.
MARKER_RE = re.compile(r'name="(\w+?)_aruco_(\d+)_(white_base|black_\d+_\d+|marker)"')
BASE_HALF_Z = 0.001


def _marker_geom(prefix: str, marker_id: int, base_size: float, z_base: float) -> str:
    half = base_size / 2.0
    return (
        f'      <geom name="{prefix}_aruco_{marker_id}_marker" type="box" '
        f'size="{half:.5f} {half:.5f} {BASE_HALF_Z:g}" pos="0 0 {z_base:.5f}" '
        f'material="stretch_assist_aruco_{marker_id}_mat" '
        f'contype="0" conaffinity="0" />'
    )


def regenerate(
    base_size: float,
    size_by_prefix: dict[str, float] | None = None,
    dry_run: bool = False,
) -> None:
    size_by_prefix = size_by_prefix or {}
    text = SCENE.read_text()
    lines = text.splitlines()

    groups: dict[tuple[str, int], list[int]] = {}
    order: list[tuple[str, int]] = []
    for i, line in enumerate(lines):
        m = MARKER_RE.search(line)
        if not m:
            continue
        key = (m.group(1), int(m.group(2)))
        if key not in groups:
            groups[key] = []
            order.append(key)
        groups[key].append(i)

    if not order:
        raise SystemExit("No ArUco marker geoms found in scene.xml")

    for key in reversed(order):
        prefix, marker_id = key
        idxs = groups[key]
        start, end = idxs[0], idxs[-1]
        if idxs != list(range(start, end + 1)):
            raise SystemExit(f"Marker {prefix}_aruco_{marker_id} geoms are not contiguous")

        z_base = float(re.search(r'pos="0 0 ([-\d.]+)"', lines[start]).group(1))
        size = size_by_prefix.get(prefix, base_size)
        new_line = _marker_geom(prefix, marker_id, size, z_base)
        print(
            f"{prefix}_aruco_{marker_id}: {end - start + 1} geom(s) -> 1 textured box "
            f"(base {size * 100:.1f} cm, z={z_base:.3f})"
        )
        lines[start : end + 1] = [new_line]

    out = "\n".join(lines) + ("\n" if text.endswith("\n") else "")
    if dry_run:
        print("\n[dry-run] scene.xml not modified")
        return
    SCENE.write_text(out)
    print(f"\nWrote {SCENE}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Use textured-box ArUco markers in scene.xml.")
    parser.add_argument("--base-size", type=float, default=0.05, help="Full width (m) of the marker square (default for all objects).")
    parser.add_argument(
        "--size",
        action="append",
        default=[],
        metavar="PREFIX=METERS",
        help="Per-object size override, e.g. --size glass=0.03 (repeatable).",
    )
    parser.add_argument("--dry-run", action="store_true", help="Preview without writing.")
    args = parser.parse_args()

    size_by_prefix: dict[str, float] = {}
    for item in args.size:
        prefix, _, value = item.partition("=")
        if not value:
            parser.error(f"Invalid --size '{item}', expected PREFIX=METERS")
        size_by_prefix[prefix.strip()] = float(value)

    regenerate(args.base_size, size_by_prefix=size_by_prefix, dry_run=args.dry_run)


if __name__ == "__main__":
    main()
