"""Deliver an object, then look at the destination to check it actually landed.

Runs a full mission, then aims the head at the just-used station and reports
whether the object's marker is detected there (i.e. it was placed, not dropped).
"""

import os

os.environ.setdefault("USE_SIM", "1")
os.environ.setdefault("STRETCH_SIM_CAMERA_HZ", "12")

import time

from perception import detect_objects, target_id_for_name
from state_machine import StretchAssistStateMachine


def main() -> None:
    from stretch_toolkit import HEAD_CAMERA, WRIST_CAMERA, controller

    machine = StretchAssistStateMachine(
        controller=controller,
        head_camera=HEAD_CAMERA,
        wrist_camera=WRIST_CAMERA,
        config_path="stretch_assist_config.json",
        debug_perception=True,
    )
    time.sleep(2.0)
    tid = target_id_for_name("medicine_box")
    machine.request("medicine_box", destination="table")
    state = machine.run(max_runtime_s=160.0, close_vision=False)
    print(f"[verify] mission ended: {state.value}; medicine memory x={machine._object_base_x(tid):.2f}")

    # Aim the head down at the station and look for the medicine marker there.
    t0 = time.time()
    seen = False
    while time.time() - t0 < 8.0:
        machine._drive_joints_to(
            {"head_pan_counterclockwise": -1.5, "head_tilt_up": -0.85}, machine.config.section("approach")
        )
        cmd, _ = machine._drive_joints_to(
            {"head_pan_counterclockwise": -1.5, "head_tilt_up": -0.85}, machine.config.section("approach")
        )
        machine._send_command(cmd)
        frame, depth = HEAD_CAMERA.rgb_cam.get_frame(), HEAD_CAMERA.depth_cam.get_frame()
        dets = detect_objects(frame, depth, camera_info=HEAD_CAMERA, target_ids={tid})
        if dets:
            seen = True
            print(f"[verify] medicine marker FOUND at the station -> it was placed (no drop)")
            break
        time.sleep(1.0 / 30.0)
    if not seen:
        print("[verify] medicine marker NOT found at the station -> it likely dropped/missed")


if __name__ == "__main__":
    main()
