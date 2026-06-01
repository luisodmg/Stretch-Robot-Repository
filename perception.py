"""ArUco-based object perception for Stretch Assist.

The functions in this module are intentionally backend-neutral. They operate on
RGB/depth arrays and optional camera info objects from ``stretch_toolkit`` so the
same code can run against the MuJoCo digital twin or a physical Stretch.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Mapping, Sequence

import cv2
import numpy as np


TARGET_OBJECTS: dict[int, str] = {
    0: "medicine_box",
    1: "glass",
    2: "tissue",
}

TARGET_LABELS: dict[int, str] = {
    0: "Medicine box",
    1: "Glass",
    2: "Tissue",
}


@dataclass(frozen=True)
class DetectedObject:
    """Single ArUco target detection with optional 3D camera-frame position."""

    name: str
    aruco_id: int
    centroid_px: tuple[float, float]
    depth_m: float | None
    position_camera_m: tuple[float, float, float] | None
    marker_area_px: float
    corners_px: tuple[tuple[float, float], ...]

    @property
    def label(self) -> str:
        return TARGET_LABELS.get(self.aruco_id, self.name.replace("_", " ").title())

    @property
    def position_3d(self) -> tuple[float, float, float] | None:
        """Alias used by the requirements document."""

        return self.position_camera_m


def target_id_for_name(target: str | int) -> int:
    """Return the ArUco ID for a target name, label, or integer ID."""

    if isinstance(target, int):
        if target in TARGET_OBJECTS:
            return target
        raise ValueError(f"Unknown Stretch Assist target ID: {target}")

    normalized = target.strip().lower().replace(" ", "_").replace("-", "_")
    for aruco_id, name in TARGET_OBJECTS.items():
        if normalized in {name, TARGET_LABELS[aruco_id].lower().replace(" ", "_")}:
            return aruco_id

    valid = ", ".join(TARGET_OBJECTS.values())
    raise ValueError(f"Unknown Stretch Assist target '{target}'. Expected one of: {valid}")


def object_name_for_id(aruco_id: int) -> str:
    """Return the canonical object name for an ArUco ID."""

    try:
        return TARGET_OBJECTS[aruco_id]
    except KeyError as exc:
        raise ValueError(f"ArUco ID {aruco_id} is not in the Stretch Assist vocabulary") from exc


def detect_object(
    frame: np.ndarray | None,
    depth_frame: np.ndarray | None = None,
    *,
    camera_info=None,
    camera_matrix: np.ndarray | None = None,
    target_id: int | None = None,
    target_ids: Iterable[int] | None = None,
    dictionary_name: str = "DICT_4X4_50",
    depth_sample_radius: int = 3,
    depth_scale: float | None = None,
) -> DetectedObject | None:
    """Detect the best matching Stretch Assist target in an RGB frame.

    Args:
        frame: RGB or BGR image containing the marker.
        depth_frame: Depth image aligned or projectable from the RGB image.
        camera_info: Optional ``stretch_toolkit.DepthCamInfo`` or compatible
            object exposing camera intrinsics.
        camera_matrix: Optional 3x3 RGB camera intrinsic matrix. Ignored when
            ``camera_info`` supplies intrinsics.
        target_id: Single ArUco ID to search for.
        target_ids: Allowed marker IDs. Defaults to IDs 0, 1, and 2.
        dictionary_name: OpenCV ArUco dictionary name.
        depth_sample_radius: Pixel radius for median depth sampling.
        depth_scale: Optional depth-unit-to-meter scale. If omitted, integer
            depth images use ``camera_info.depth_scale`` or 0.001, and floating
            depth images are treated as meters.

    Returns:
        ``DetectedObject`` or ``None`` when no requested marker is visible.
    """

    detections = detect_objects(
        frame,
        depth_frame,
        camera_info=camera_info,
        camera_matrix=camera_matrix,
        target_id=target_id,
        target_ids=target_ids,
        dictionary_name=dictionary_name,
        depth_sample_radius=depth_sample_radius,
        depth_scale=depth_scale,
    )
    return detections[0] if detections else None


def detect_objects(
    frame: np.ndarray | None,
    depth_frame: np.ndarray | None = None,
    *,
    camera_info=None,
    camera_matrix: np.ndarray | None = None,
    target_id: int | None = None,
    target_ids: Iterable[int] | None = None,
    dictionary_name: str = "DICT_4X4_50",
    depth_sample_radius: int = 3,
    depth_scale: float | None = None,
) -> list[DetectedObject]:
    """Detect all visible Stretch Assist targets, largest marker first."""

    if frame is None:
        return []

    aruco = _require_aruco()
    dictionary = _get_dictionary(aruco, dictionary_name)
    parameters = _get_detector_parameters(aruco)
    gray = _to_grayscale(frame)
    corners, ids = _detect_markers(aruco, gray, dictionary, parameters)

    if ids is None or len(ids) == 0:
        return []

    allowed_ids = _allowed_target_ids(target_id=target_id, target_ids=target_ids)
    rgb_intrinsics = _camera_matrix_from(camera_info, camera_matrix)
    detections: list[DetectedObject] = []

    for raw_id, raw_corners in zip(ids.flatten(), corners):
        aruco_id = int(raw_id)
        if aruco_id not in allowed_ids:
            continue

        marker_corners = np.asarray(raw_corners, dtype=float).reshape(4, 2)
        centroid = _marker_centroid(marker_corners)
        marker_area = float(abs(cv2.contourArea(marker_corners.astype(np.float32))))
        depth_m = _sample_depth_meters(
            depth_frame,
            centroid,
            camera_info=camera_info,
            depth_scale=depth_scale,
            sample_radius=depth_sample_radius,
        )
        position = _position_from_depth(centroid, depth_m, rgb_intrinsics)

        detections.append(
            DetectedObject(
                name=object_name_for_id(aruco_id),
                aruco_id=aruco_id,
                centroid_px=centroid,
                depth_m=depth_m,
                position_camera_m=position,
                marker_area_px=marker_area,
                corners_px=tuple((float(x), float(y)) for x, y in marker_corners),
            )
        )

    detections.sort(key=lambda item: item.marker_area_px, reverse=True)
    return detections


def draw_detections(
    frame: np.ndarray,
    detections: Sequence[DetectedObject],
    *,
    color: tuple[int, int, int] = (0, 255, 0),
) -> np.ndarray:
    """Return a copy of ``frame`` annotated with target centers and labels."""

    annotated = frame.copy()
    for detection in detections:
        corners = np.asarray(detection.corners_px, dtype=np.int32).reshape((-1, 1, 2))
        cv2.polylines(annotated, [corners], isClosed=True, color=color, thickness=2)
        center = tuple(int(v) for v in detection.centroid_px)
        cv2.circle(annotated, center, 4, color, -1)
        cv2.putText(
            annotated,
            f"{detection.label} ({detection.aruco_id})",
            (center[0] + 8, center[1] - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            color,
            1,
            cv2.LINE_AA,
        )
    return annotated


def _require_aruco():
    aruco = getattr(cv2, "aruco", None)
    if aruco is None:
        raise RuntimeError(
            "OpenCV ArUco support is unavailable. Install an OpenCV build with cv2.aruco."
        )
    return aruco


def _get_dictionary(aruco, dictionary_name: str):
    dictionary_id = getattr(aruco, dictionary_name, None)
    if dictionary_id is None:
        raise ValueError(f"Unknown ArUco dictionary: {dictionary_name}")

    if hasattr(aruco, "getPredefinedDictionary"):
        return aruco.getPredefinedDictionary(dictionary_id)
    return aruco.Dictionary_get(dictionary_id)


def _get_detector_parameters(aruco):
    if hasattr(aruco, "DetectorParameters"):
        return aruco.DetectorParameters()
    return aruco.DetectorParameters_create()


def _detect_markers(aruco, gray, dictionary, parameters):
    if hasattr(aruco, "ArucoDetector"):
        detector = aruco.ArucoDetector(dictionary, parameters)
        corners, ids, _ = detector.detectMarkers(gray)
        return corners, ids

    corners, ids, _ = aruco.detectMarkers(gray, dictionary, parameters=parameters)
    return corners, ids


def _to_grayscale(frame: np.ndarray) -> np.ndarray:
    if frame.ndim == 2:
        return frame
    if frame.shape[2] == 4:
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2GRAY)
    return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


def _allowed_target_ids(
    *, target_id: int | None, target_ids: Iterable[int] | None
) -> set[int]:
    if target_id is not None:
        return {target_id}
    if target_ids is not None:
        return {int(item) for item in target_ids}
    return set(TARGET_OBJECTS)


def _marker_centroid(corners: np.ndarray) -> tuple[float, float]:
    center = corners.mean(axis=0)
    return float(center[0]), float(center[1])


def _camera_matrix_from(camera_info, camera_matrix: np.ndarray | None) -> np.ndarray | None:
    if camera_info is not None and getattr(camera_info, "camera_matrix", None) is not None:
        return np.asarray(camera_info.camera_matrix, dtype=float)
    if camera_matrix is not None:
        return np.asarray(camera_matrix, dtype=float)
    return None


def _sample_depth_meters(
    depth_frame: np.ndarray | None,
    centroid: tuple[float, float],
    *,
    camera_info,
    depth_scale: float | None,
    sample_radius: int,
) -> float | None:
    if depth_frame is None:
        return None

    depth_image = np.asarray(depth_frame)
    if depth_image.size == 0:
        return None

    x, y = _depth_pixel_from_rgb_centroid(centroid, camera_info)
    height, width = depth_image.shape[:2]
    x = int(np.clip(round(x), 0, width - 1))
    y = int(np.clip(round(y), 0, height - 1))
    radius = max(0, int(sample_radius))

    y_min = max(0, y - radius)
    y_max = min(height, y + radius + 1)
    x_min = max(0, x - radius)
    x_max = min(width, x + radius + 1)
    samples = depth_image[y_min:y_max, x_min:x_max].astype(float).reshape(-1)
    samples = samples[np.isfinite(samples)]
    samples = samples[samples > 0]
    if samples.size == 0:
        return None

    scale = _depth_scale_for(depth_image, camera_info, depth_scale)
    return float(np.median(samples) * scale)


def _depth_pixel_from_rgb_centroid(
    centroid: tuple[float, float], camera_info
) -> tuple[float, float]:
    if camera_info is None:
        return centroid

    attrs = ("fx", "fy", "cx", "cy", "depth_fx", "depth_fy", "depth_cx", "depth_cy")
    if not all(hasattr(camera_info, attr) for attr in attrs):
        return centroid

    x_norm = (centroid[0] - camera_info.cx) / camera_info.fx
    y_norm = (centroid[1] - camera_info.cy) / camera_info.fy
    return (
        x_norm * camera_info.depth_fx + camera_info.depth_cx,
        y_norm * camera_info.depth_fy + camera_info.depth_cy,
    )


def _depth_scale_for(
    depth_image: np.ndarray, camera_info, explicit_depth_scale: float | None
) -> float:
    if explicit_depth_scale is not None:
        return explicit_depth_scale
    if np.issubdtype(depth_image.dtype, np.floating):
        return 1.0
    if camera_info is not None and getattr(camera_info, "depth_scale", None) is not None:
        return float(camera_info.depth_scale)
    return 0.001


def _position_from_depth(
    centroid: tuple[float, float],
    depth_m: float | None,
    camera_matrix: np.ndarray | None,
) -> tuple[float, float, float] | None:
    if depth_m is None or camera_matrix is None:
        return None

    fx = float(camera_matrix[0, 0])
    fy = float(camera_matrix[1, 1])
    cx = float(camera_matrix[0, 2])
    cy = float(camera_matrix[1, 2])
    if fx == 0 or fy == 0:
        return None

    x_m = (centroid[0] - cx) * depth_m / fx
    y_m = (centroid[1] - cy) * depth_m / fy
    return float(x_m), float(y_m), float(depth_m)
