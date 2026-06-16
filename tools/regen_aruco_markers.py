#!/usr/bin/env python3
"""Regenerate the Stretch Assist ArUco marker geoms in ``scene.xml``.

The markers are drawn in MuJoCo as a stack of box geoms: one white base plus
one black box per black ArUco cell. Hand-editing ~29 geoms per marker is
error-prone, so this script rebuilds them from the OpenCV dictionary (which
guarantees the correct marker ID pattern) at a chosen physical size.

Geometry model (matches the original hand-authored markers):
- The white base is 8 cells wide: a 6x6 ArUco grid (4x4 data + 1 black border
  ring) with a 1-cell white quiet zone on every side.
- ``cell = base_size / 8``; grid cell centers sit at ``cell * [2.5, 1.5, 0.5,
  -0.5, -1.5, -2.5]`` on both axes.
- Each marker keeps its existing Z placement (height on the object) and the
  small Z offset that lifts the black tiles above the white base.

Usage:
    python tools/regen_aruco_markers.py --base-size 0.05
    python tools/regen_aruco_markers.py --base-size 0.05 --dry-run
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path

import cv2
import numpy as np

SCENE = Path(__file__).resolve().parent.parent / "stretch_mujoco" / "models" / "scene.xml"
DICTIONARY = "DICT_4X4_50"

# White base / black tile thickness (half-extent in Z), unchanged from original.
BASE_HALF_Z = 0.001
TILE_HALF_Z = 0.0015

# prefix -> ArUco id, parsed from geom names like "<prefix>_aruco_<id>_white_base".
MARKER_RE = re.compile(r'name="(\w+?)_aruco_(\d+)_(white_base|black_\d+_\d+)"')


def _cell_pattern(aruco, marker_id: int) -> np.ndarray:
    """Return the 6x6 cell grid (True = black) for ``marker_id``."""

    dictionary = (
        aruco.getPredefinedDictionary(getattr(aruco, DICTIONARY))
        if hasattr(aruco, "getPredefinedDictionary")
        else aruco.Dictionary_get(getattr(aruco, DICTIONARY))
    )
    # One pixel per cell: 4x4 data + 1 border bit on each side -> 6x6.
    img = aruco.generateImageMarker(dictionary, marker_id, 6, borderBits=1)
    return img == 0


def _marker_geoms(prefix: str, marker_id: int, base_size: float, z_base: float) -> list[str]:
    """Build the white-base + black-cell geom XML lines for one marker."""

    cell = base_size / 8.0
    base_half = base_size / 2.0
    tile_half = cell / 2.0
    tile_z = z_base + 0.003
    coords = [cell * (2.5 - i) for i in range(6)]  # row/col index -> x/y center

    indent = "      "
    lines = [
        f'{indent}<geom name="{prefix}_aruco_{marker_id}_white_base" type="box" '
        f'size="{base_half:.5f} {base_half:.5f} {BASE_HALF_Z:g}" '
        f'pos="0 0 {z_base:.5f}" material="stretch_assist_marker_white" '
        f'contype="0" conaffinity="0" />'
    ]

    aruco = cv2.aruco
    pattern = _cell_pattern(aruco, marker_id)
    for r in range(6):
        for c in range(6):
            if not pattern[r, c]:
                continue
            x, y = coords[r], coords[c]
            lines.append(
                f'{indent}<geom name="{prefix}_aruco_{marker_id}_black_{r + 1}_{c + 1}" '
                f'type="box" size="{tile_half:.5f} {tile_half:.5f} {TILE_HALF_Z:g}" '
                f'pos="{x:.5f} {y:.5f} {tile_z:.5f}" '
                f'material="stretch_assist_marker_black" contype="0" conaffinity="0" />'
            )
    return lines


def regenerate(base_size: float, dry_run: bool = False) -> None:
    text = SCENE.read_text()
    lines = text.splitlines()

    # Group marker geom line indices by (prefix, id), in order of appearance.
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

    # Rebuild each contiguous block, replacing from the bottom up so earlier
    # line indices stay valid.
    for key in reversed(order):
        prefix, marker_id = key
        idxs = groups[key]
        start, end = idxs[0], idxs[-1]
        if idxs != list(range(start, end + 1)):
            raise SystemExit(f"Marker {prefix}_aruco_{marker_id} geoms are not contiguous")

        # z of the white base preserves the marker's height on the object.
        z_base = float(re.search(r'pos="0 0 ([-\d.]+)"', lines[start]).group(1))
        new_lines = _marker_geoms(prefix, marker_id, base_size, z_base)
        print(
            f"{prefix}_aruco_{marker_id}: {end - start + 1} -> {len(new_lines)} geoms "
            f"(base {base_size * 100:.1f} cm, z={z_base:.3f})"
        )
        lines[start : end + 1] = new_lines

    out = "\n".join(lines) + ("\n" if text.endswith("\n") else "")
    if dry_run:
        print("\n[dry-run] scene.xml not modified")
        return
    SCENE.write_text(out)
    print(f"\nWrote {SCENE}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Regenerate ArUco marker geoms in scene.xml.")
    parser.add_argument(
        "--base-size",
        type=float,
        default=0.05,
        help="Full width (m) of the white base square. Original was 0.18.",
    )
    parser.add_argument("--dry-run", action="store_true", help="Preview without writing.")
    args = parser.parse_args()
    regenerate(args.base_size, dry_run=args.dry_run)


if __name__ == "__main__":
    main()
