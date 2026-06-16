"""Live "robot vision" window for Stretch Assist.

Shows what the robot is currently seeing through its head (D435i) and wrist
(D405) cameras, with the ArUco detections drawn on top: bounding box, marker
ID, object label and measured distance. A header banner shows the current
state machine state and the requested target.

This is meant for demos: it mirrors what a real Stretch would perceive and how
the ArUco-based perception drives the behaviour, so an audience can follow along.

The window must be driven from the main thread (OpenCV GUI requirement). The
state machine calls :meth:`VisionWindow.render` once per control loop.
"""

from __future__ import annotations

from typing import Sequence

import cv2
import numpy as np

from perception import DetectedObject, TARGET_LABELS, detect_objects


# BGR colors.
_COLOR_TARGET = (0, 220, 0)       # green: the marker we are looking for
_COLOR_OTHER = (0, 180, 255)      # amber: other known markers
_COLOR_TEXT = (255, 255, 255)
_COLOR_BANNER = (40, 40, 40)
# Single-camera (head only) panel is shown large; with two cameras each panel
# is smaller so both fit side by side.
_PANEL_W_SINGLE = 760
_PANEL_H_SINGLE = 560
_PANEL_W_DUAL = 480
_PANEL_H_DUAL = 360


class VisionWindow:
    """Composites head + wrist camera feeds with ArUco overlays."""

    def __init__(
        self,
        head_camera=None,
        wrist_camera=None,
        *,
        target_id: int | None = None,
        dictionary_name: str = "DICT_4X4_50",
        depth_sample_radius: int = 3,
        window_name: str = "Stretch Assist - Robot Vision",
    ):
        self.head_camera = head_camera
        self.wrist_camera = wrist_camera
        self.target_id = target_id
        self.dictionary_name = dictionary_name
        self.depth_sample_radius = depth_sample_radius
        self.window_name = window_name
        self._created = False
        self._closed = False
        self._dual = wrist_camera is not None
        self._panel_w = _PANEL_W_DUAL if self._dual else _PANEL_W_SINGLE
        self._panel_h = _PANEL_H_DUAL if self._dual else _PANEL_H_SINGLE

    # -- public API ---------------------------------------------------------

    def render(self, *, state: str = "", message: str | None = None) -> bool:
        """Pull fresh frames, draw detections, and show one window frame.

        Returns ``False`` when the user closed the window or pressed ESC, so the
        caller can stop driving it. Never raises on a camera/render hiccup.
        """

        if self._closed:
            return False

        try:
            panels = [self._build_panel(self.head_camera, "HEAD CAMERA (D435i)")]
            if self._dual:
                panels.append(self._build_panel(self.wrist_camera, "WRIST CAMERA (D405)"))
            canvas = self._compose(panels, state, message)

            if not self._created:
                cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
                self._created = True
            cv2.imshow(self.window_name, canvas)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                self.close()
                return False
            # Window closed via the window manager 'X' button.
            if cv2.getWindowProperty(self.window_name, cv2.WND_PROP_VISIBLE) < 1:
                self.close()
                return False
        except Exception as exc:  # pragma: no cover - GUI/runtime safety net
            print(f"[Vision] render skipped: {exc}")
        return True

    def close(self) -> None:
        if self._created and not self._closed:
            try:
                cv2.destroyWindow(self.window_name)
            except Exception:
                pass
        self._closed = True

    # -- internals ----------------------------------------------------------

    def _build_panel(self, camera, title: str) -> np.ndarray:
        frame = self._grab_rgb(camera)
        depth = self._grab_depth(camera)

        if frame is None:
            panel = self._placeholder(title, "no signal")
            return panel

        bgr = self._to_bgr(frame)
        detections: Sequence[DetectedObject] = []
        try:
            detections = detect_objects(
                frame,
                depth,
                camera_info=camera,
                dictionary_name=self.dictionary_name,
                depth_sample_radius=self.depth_sample_radius,
            )
        except Exception:
            detections = []

        annotated = self._draw(bgr, detections)
        annotated = cv2.resize(
            annotated, (self._panel_w, self._panel_h), interpolation=cv2.INTER_AREA
        )
        self._panel_header(annotated, title, detections)
        return annotated

    def _draw(self, bgr: np.ndarray, detections: Sequence[DetectedObject]) -> np.ndarray:
        out = bgr.copy()
        for det in detections:
            is_target = self.target_id is None or det.aruco_id == self.target_id
            color = _COLOR_TARGET if is_target else _COLOR_OTHER
            corners = np.asarray(det.corners_px, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(out, [corners], isClosed=True, color=color, thickness=2)
            cx, cy = (int(v) for v in det.centroid_px)
            cv2.circle(out, (cx, cy), 4, color, -1)
            label = TARGET_LABELS.get(det.aruco_id, det.name)
            dist = f" {det.depth_m:.2f} m" if det.depth_m is not None else ""
            text = f"{label} [id {det.aruco_id}]{dist}"
            self._label(out, text, (cx + 8, cy - 8), color)
        return out

    def _panel_header(self, panel: np.ndarray, title: str, detections) -> None:
        cv2.rectangle(panel, (0, 0), (panel.shape[1], 26), _COLOR_BANNER, -1)
        seen = "  detecting: " + (
            ", ".join(str(d.aruco_id) for d in detections) if detections else "none"
        )
        self._label(panel, title + seen, (8, 18), _COLOR_TEXT, scale=0.5)

    def _compose(self, panels, state: str, message: str | None) -> np.ndarray:
        banner_h = 56
        body = np.hstack(panels)
        canvas = np.zeros((body.shape[0] + banner_h, body.shape[1], 3), dtype=np.uint8)
        canvas[banner_h:, :] = body

        cv2.rectangle(canvas, (0, 0), (canvas.shape[1], banner_h), (20, 20, 20), -1)
        target = (
            TARGET_LABELS.get(self.target_id, str(self.target_id))
            if self.target_id is not None
            else "-"
        )
        self._label(canvas, "STRETCH ASSIST", (12, 24), (0, 220, 0), scale=0.7, thick=2)
        line = f"State: {state or '-'}    Target: {target}"
        if message:
            line += f"    ({message})"
        self._label(canvas, line, (12, 46), _COLOR_TEXT, scale=0.55)
        return canvas

    # -- frame helpers ------------------------------------------------------

    @staticmethod
    def _grab_rgb(camera):
        if camera is None:
            return None
        try:
            if hasattr(camera, "rgb_cam"):
                return camera.rgb_cam.get_frame()
            if hasattr(camera, "get_frame"):
                return camera.get_frame()
        except Exception:
            return None
        return None

    @staticmethod
    def _grab_depth(camera):
        if camera is None:
            return None
        try:
            if hasattr(camera, "depth_cam"):
                return camera.depth_cam.get_frame()
        except Exception:
            return None
        return None

    @staticmethod
    def _to_bgr(frame: np.ndarray) -> np.ndarray:
        arr = np.asarray(frame)
        if arr.ndim == 2:
            return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
        if arr.shape[2] == 4:
            return cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
        # MuJoCo / RealSense RGB -> BGR for correct colors in OpenCV.
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)

    def _placeholder(self, title: str, msg: str) -> np.ndarray:
        panel = np.full((self._panel_h, self._panel_w, 3), 30, dtype=np.uint8)
        cv2.rectangle(panel, (0, 0), (self._panel_w, 26), _COLOR_BANNER, -1)
        self._label(panel, title, (8, 18), _COLOR_TEXT, scale=0.5)
        self._label(
            panel, msg, (self._panel_w // 2 - 50, self._panel_h // 2), (120, 120, 120), scale=0.7
        )
        return panel

    @staticmethod
    def _label(img, text, org, color, scale: float = 0.45, thick: int = 1) -> None:
        # Draw a dark outline first for readability over any background.
        cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thick + 2, cv2.LINE_AA)
        cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)
