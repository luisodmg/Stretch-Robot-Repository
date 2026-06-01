import numpy as np
import pytest

import perception


class FakeDepthCamera:
    fx = 100.0
    fy = 100.0
    cx = 100.0
    cy = 100.0
    depth_fx = 100.0
    depth_fy = 100.0
    depth_cx = 100.0
    depth_cy = 100.0
    depth_scale = 0.001
    camera_matrix = np.array(
        [
            [100.0, 0.0, 100.0],
            [0.0, 100.0, 100.0],
            [0.0, 0.0, 1.0],
        ]
    )


def _aruco_marker(marker_id: int, size: int = 120):
    aruco = getattr(perception.cv2, "aruco", None)
    if aruco is None:
        pytest.skip("cv2.aruco is unavailable")

    dictionary = perception._get_dictionary(aruco, "DICT_4X4_50")
    if hasattr(aruco, "generateImageMarker"):
        return aruco.generateImageMarker(dictionary, marker_id, size)
    marker = np.zeros((size, size), dtype=np.uint8)
    aruco.drawMarker(dictionary, marker_id, size, marker, 1)
    return marker


def test_detect_object_returns_name_id_and_3d_position():
    marker = _aruco_marker(0)
    frame = np.full((240, 240, 3), 255, dtype=np.uint8)
    frame[40:160, 40:160] = np.dstack([marker, marker, marker])
    depth = np.full((240, 240), 1200, dtype=np.uint16)

    detected = perception.detect_object(
        frame,
        depth,
        camera_info=FakeDepthCamera(),
        target_id=0,
    )

    assert detected is not None
    assert detected.name == "medicine_box"
    assert detected.aruco_id == 0
    assert detected.depth_m == pytest.approx(1.2)
    assert detected.position_3d == pytest.approx((-0.006, -0.006, 1.2), abs=0.02)


def test_detect_object_ignores_unknown_targets():
    marker = _aruco_marker(4)
    frame = np.full((220, 220, 3), 255, dtype=np.uint8)
    frame[50:170, 50:170] = np.dstack([marker, marker, marker])

    assert perception.detect_object(frame, target_ids={0, 1, 2}) is None
