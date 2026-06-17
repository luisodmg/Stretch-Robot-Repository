"""Run several deliveries back-to-back the way the interactive loop does.

Goes straight to each object's remembered location (no trip back to start) and
exercises re-grabbing an object from the station it was left at.
"""

import os

os.environ.setdefault("USE_SIM", "1")
os.environ.setdefault("STRETCH_SIM_CAMERA_HZ", "12")

import time

from perception import target_id_for_name
from state_machine import StretchAssistStateMachine


# glass: pickup -> table, then table -> shelf (re-grab from the station),
# then medicine_box: pickup -> person (coming from the shelf, no homing).
MISSIONS = [("glass", "table"), ("glass", "shelf"), ("medicine_box", "person")]


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

    for obj, dest in MISSIONS:
        tid = target_id_for_name(obj)
        pre = machine.preapproach_pose(tid)
        print(f"[itest] fetching {obj} (id {tid}) from remembered x={machine._object_base_x(tid):.2f}")
        machine.drive_base_to(pre, timeout_s=45.0)
        print(f"[itest] at preapproach {machine._read_base_pose()}")
        machine.request(obj, destination=dest)
        state = machine.run(max_runtime_s=160.0, close_vision=False)
        print(f"[itest] MISSION {obj} -> {dest}: {state.value}")

    print("[itest] all missions done")


if __name__ == "__main__":
    main()
