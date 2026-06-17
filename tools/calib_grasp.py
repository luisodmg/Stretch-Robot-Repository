"""Measure the gripper position at the grasp pose vs each object's known spot.

Drives the base to each object's stop_base_x and the arm to the grasp pose, then
reads the gripper (end-effector) world pose from the sim and prints how far it is
from the object. Use the printed err_x/err_y to correct stop_base_x and arm_out
so the grasp actually centers on the object.
"""

import os

os.environ.setdefault("USE_SIM", "1")
os.environ.setdefault("STRETCH_SIM_CAMERA_HZ", "12")

import time

from state_machine import BasePose, StretchAssistStateMachine

OBJ = {0: (0.20, -0.70), 1: (0.35, -0.70), 2: (0.50, -0.70)}  # known x, y


def main() -> None:
    from stretch_toolkit import HEAD_CAMERA, controller

    machine = StretchAssistStateMachine(
        controller=controller,
        head_camera=HEAD_CAMERA,
        config_path="stretch_assist_config.json",
    )
    machine._read_base_pose()  # force the lazy controller/sim to start
    time.sleep(2.0)
    sim = controller.sim  # the StretchMujocoSimulator (proxied through controller)

    for tid, (ox, oy) in OBJ.items():
        machine.target_id = tid
        cfg = machine.config.section("manipulation")
        stop_x = machine._object_base_x(tid)
        # Drive straight forward to stop_x precisely (mimics APPROACH), keeping
        # heading ~0 (the robot starts facing +x and we never command a turn).
        t0 = time.time()
        while time.time() - t0 < 30.0:
            pose = machine._read_base_pose()
            if pose is not None and pose.x >= stop_x - 0.01:
                break
            machine._send_command({"base_forward": 0.2})
            time.sleep(1.0 / 30.0)
        machine._send_command({})

        grasp = {**dict(cfg["pregrasp"]), **dict(cfg["over_pose"]), **machine._grasp_pose(cfg)}
        # Drive to the grasp pose AND hold it steady so the arm fully settles
        # before measuring (otherwise the reading is taken mid-motion).
        t0 = time.time()
        while time.time() - t0 < 16.0:
            cmd, _ = machine._drive_joints_to(grasp, cfg)
            machine._send_command(cmd)
            st = machine._read_state() if hasattr(machine, "_read_state") else machine.controller.get_state()
            settled = (
                abs(st.get("arm_out", 0) - grasp["arm_out"]) < 0.005
                and abs(st.get("lift_up", 0) - grasp["lift_up"]) < 0.005
            )
            if settled and time.time() - t0 > 4.0:
                break
            time.sleep(1.0 / 30.0)
        time.sleep(1.0)

        st = machine.controller.get_state()
        ee = sim.get_ee_pose()
        gx, gy, gz = float(ee[0, 3]), float(ee[1, 3]), float(ee[2, 3])
        base = machine._read_base_pose()
        print(
            f"[calib] id {tid} base=({base.x:.3f},{base.y:.3f},{base.theta:.3f}) "
            f"arm_out={st.get('arm_out'):.3f} lift={st.get('lift_up'):.3f} "
            f"target=({ox:.3f},{oy:.3f}) gripper=({gx:.3f},{gy:.3f},{gz:.3f}) "
            f"err_x={ox - gx:+.3f} err_y={oy - gy:+.3f}"
        )


if __name__ == "__main__":
    main()
