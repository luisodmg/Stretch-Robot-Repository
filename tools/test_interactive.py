"""Run several deliveries back-to-back with homing between them.

Exercises the interactive core (drive_base_to home + repeated request/run) and
all three objects in one session, without needing the GUI selector.
"""

import os

os.environ.setdefault("USE_SIM", "1")
os.environ.setdefault("STRETCH_SIM_CAMERA_HZ", "12")

import time

from state_machine import StretchAssistStateMachine


MISSIONS = [("glass", "table"), ("medicine_box", "shelf"), ("tissue", "person")]


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
    home = machine._read_base_pose()
    print(f"[itest] home={home}")

    for obj, dest in MISSIONS:
        if home is not None:
            machine.drive_base_to(home, timeout_s=40.0)
            print(f"[itest] homed to {machine._read_base_pose()}")
        machine.request(obj, destination=dest)
        state = machine.run(max_runtime_s=150.0, close_vision=False)
        print(f"[itest] MISSION {obj} -> {dest}: {state.value}")

    print("[itest] all missions done")


if __name__ == "__main__":
    main()
