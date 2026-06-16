"""Run one pick and save head + wrist camera frames at the grasp moment.

Lets us see how the gripper is actually holding the object so the grasp poses
can be tuned. Saves to /tmp/grasp_<cam>_<n>.png.
"""

import os
import sys

os.environ.setdefault("USE_SIM", "1")
os.environ.setdefault("STRETCH_SIM_CAMERA_HZ", "12")

import time

import cv2
import numpy as np

from state_machine import AssistState, StretchAssistStateMachine


def _save(tag, frame):
    if frame is None:
        return
    arr = np.asarray(frame)
    bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR) if arr.ndim == 3 else arr
    cv2.imwrite(f"/tmp/grasp_{tag}.png", bgr)
    print(f"[grasp] saved /tmp/grasp_{tag}.png {arr.shape}")


def main() -> None:
    target = sys.argv[1] if len(sys.argv) > 1 else "glass"
    from stretch_toolkit import HEAD_CAMERA, WRIST_CAMERA, controller

    machine = StretchAssistStateMachine(
        controller=controller,
        head_camera=HEAD_CAMERA,
        wrist_camera=WRIST_CAMERA,
        config_path="stretch_assist_config.json",
    )
    time.sleep(2.0)
    machine.request(target, destination="table")

    n = 0
    t0 = time.monotonic()
    grabbed_at = None
    while time.monotonic() - t0 < 120.0:
        state = machine.step()
        # Capture through GRASP and for ~2s into RETURN (object lifted).
        if state == AssistState.GRASP or (
            state == AssistState.RETURN and (grabbed_at is None or time.monotonic() - grabbed_at < 2.0)
        ):
            if state == AssistState.RETURN and grabbed_at is None:
                grabbed_at = time.monotonic()
            if n % 8 == 0:  # ~ every 8 steps
                try:
                    _save(f"head_{n:03d}", HEAD_CAMERA.rgb_cam.get_frame())
                    _save(f"wrist_{n:03d}", WRIST_CAMERA.rgb_cam.get_frame())
                except Exception as exc:
                    print(f"[grasp] capture error: {exc}")
            n += 1
        if state in (AssistState.RETURN,) and grabbed_at is not None and time.monotonic() - grabbed_at >= 2.0:
            print("[grasp] captured grasp window, stopping")
            break
        if state in (AssistState.COMPLETE, AssistState.ABORTED):
            print(f"[grasp] ended in {state.value}")
            break
        time.sleep(1.0 / 30.0)


if __name__ == "__main__":
    main()
