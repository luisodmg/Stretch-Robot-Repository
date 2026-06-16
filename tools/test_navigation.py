"""Drive the base to a destination in the real sim and log the pose over time.

Isolates the RETURN navigation from the (slow) grasp pipeline so we can see
whether the base actually converges on the goal and how long it takes.
"""

import os

os.environ.setdefault("USE_SIM", "1")
os.environ.setdefault("STRETCH_SIM_CAMERA_HZ", "12")

import sys
import time

from state_machine import AssistState, StretchAssistStateMachine, BasePose


def main() -> None:
    destination = sys.argv[1] if len(sys.argv) > 1 else "kitchen"
    from stretch_toolkit import controller

    machine = StretchAssistStateMachine(
        controller=controller,
        config_path="stretch_assist_config.json",
        detector=lambda *a, **k: None,
    )
    # Settle, then pretend we just grasped: record start and jump to RETURN.
    time.sleep(2.0)
    machine.target_id = 1  # any valid target so step() does not abort
    machine.target_name = "glass"
    machine.start_pose = machine._read_base_pose() or BasePose(0, 0, 0)
    machine.set_destination(destination)
    target = machine._destination_pose()
    print(f"[nav] start={machine.start_pose} target={target} dest={destination}")

    machine.state = AssistState.RETURN
    t0 = time.monotonic()
    last_log = 0.0
    while time.monotonic() - t0 < 60.0:
        state = machine.step()
        now = time.monotonic() - t0
        if now - last_log >= 1.0:
            pose = machine._read_base_pose()
            print(f"[nav] t={now:5.1f}s state={state.value} pose={pose} cmd={machine.last_command}")
            last_log = now
        if state != AssistState.RETURN:
            print(f"[nav] DONE in {now:.1f}s -> {state.value}")
            break
        time.sleep(1.0 / 30.0)
    else:
        print("[nav] TIMEOUT (60s wall) without arriving")


if __name__ == "__main__":
    main()
