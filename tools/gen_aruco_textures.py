#!/usr/bin/env python3
"""Generate ArUco marker PNG textures for the Stretch Assist objects.

Each texture is the ArUco marker (6x6 cells: 4x4 data + 1 black border ring)
centered on a white square with a 1-cell white quiet zone on every side, i.e.
8x8 cells total -- matching the proportions of the old box-tile markers. A
single textured box renders far crisper than ~16 floating black tiles, which is
what lets the markers shrink to ~5 cm and still decode.

Output: stretch_mujoco/models/assets/stretch_assist_aruco_<id>.png
"""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

ASSETS = Path(__file__).resolve().parent.parent / "stretch_mujoco" / "models" / "assets"
DICTIONARY = "DICT_4X4_50"
MARKER_IDS = (0, 1, 2)
CELL_PX = 96  # per ArUco cell; 8 cells -> 768x768 texture


def _dictionary(aruco):
    return (
        aruco.getPredefinedDictionary(getattr(aruco, DICTIONARY))
        if hasattr(aruco, "getPredefinedDictionary")
        else aruco.Dictionary_get(getattr(aruco, DICTIONARY))
    )


def generate() -> None:
    aruco = cv2.aruco
    d = _dictionary(aruco)
    ASSETS.mkdir(parents=True, exist_ok=True)
    for marker_id in MARKER_IDS:
        marker = aruco.generateImageMarker(d, marker_id, 6 * CELL_PX, borderBits=1)
        # 1-cell white quiet zone on every side -> 8x8 cells total.
        canvas = cv2.copyMakeBorder(
            marker, CELL_PX, CELL_PX, CELL_PX, CELL_PX,
            cv2.BORDER_CONSTANT, value=255,
        )
        path = ASSETS / f"stretch_assist_aruco_{marker_id}.png"
        cv2.imwrite(str(path), canvas)
        print(f"wrote {path} ({canvas.shape[1]}x{canvas.shape[0]})")


if __name__ == "__main__":
    generate()
