"""Run the real Stretch Assist app briefly and dump head-camera frames to disk.

Diagnostic helper: wraps the state machine's detection call to save the actual
RGB frames the perception sees, so we can inspect why markers are/aren't found.
"""

import os

os.environ.setdefault("USE_SIM", "1")

import cv2
import numpy as np

import perception
import state_machine as sm

_saved = {"n": 0}
_orig = sm.StretchAssistStateMachine._detect_from_camera


def _patched(self, camera, camera_name="head"):
    det, frame = _orig(self, camera, camera_name)
    if camera_name == "head" and frame is not None and _saved["n"] < 6:
        arr = np.asarray(frame)
        bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR) if arr.ndim == 3 else arr
        path = f"/tmp/head_{_saved['n']}.png"
        cv2.imwrite(path, bgr)
        dets = perception.detect_objects(arr, target_ids={0, 1, 2})
        sizes = [
            f"id{d.aruco_id}:{int(d.marker_area_px ** 0.5)}px" for d in dets
        ]
        print(
            f"[capture] {path} shape={arr.shape} dtype={arr.dtype} "
            f"mean={float(arr.mean()):.1f} detected={sizes or 'none'}"
        )
        _saved["n"] += 1
    return det, frame


if __name__ == "__main__":
    sm.StretchAssistStateMachine._detect_from_camera = _patched
    sm.run_stretch_assist(
        target="glass",
        use_teleop=False,
        debug_perception=True,
        show_vision=False,
        max_runtime_s=16,
    )
