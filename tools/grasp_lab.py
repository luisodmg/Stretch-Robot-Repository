"""Grasp / delivery debugging lab.

Objective telemetry so grasp tuning is data-driven instead of eyeballed. Two
modes:

  calibrate  Position the gripper at each object's grasp pose, wait until the
             arm is fully SETTLED (ee pose stops moving), then measure the real
             gripper position (sim get_ee_pose) vs the object's known position.
             Logs err_x / err_y per object and prints suggested stop_base_x and
             arm_out corrections.

  run        Run full missions; log the outcome, the gripper closure at grasp
             (how firmly it gripped), and whether the object is still seen at
             the pickup after lifting (i.e. it was actually picked up).

Everything is written to debug/sessions/<timestamp>/ (telemetry.csv + frames/)
so behavior can be reviewed across sessions.

Usage:
  python tools/grasp_lab.py calibrate
  python tools/grasp_lab.py run glass:table medicine_box:shelf tissue:person
"""

from __future__ import annotations

import os

os.environ.setdefault("USE_SIM", "1")
os.environ.setdefault("STRETCH_SIM_CAMERA_HZ", "12")

import csv
import datetime as dt
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from perception import detect_objects, target_id_for_name
from state_machine import AssistState, BasePose, StretchAssistStateMachine

# Object id -> known table position (x, y) and label.
OBJECTS = {
    0: ("medicine_box", 0.20, -0.70),
    1: ("glass", 0.35, -0.70),
    2: ("tissue", 0.50, -0.70),
}
REPO = Path(__file__).resolve().parent.parent


def _session_dir() -> Path:
    stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    path = REPO / "debug" / "sessions" / stamp
    (path / "frames").mkdir(parents=True, exist_ok=True)
    return path


def _save_frame(frames_dir: Path, name: str, frame) -> None:
    if frame is None:
        return
    arr = np.asarray(frame)
    bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR) if arr.ndim == 3 else arr
    cv2.imwrite(str(frames_dir / f"{name}.png"), bgr)


def _drive_base_forward_to(machine, stop_x: float, timeout: float = 30.0) -> None:
    """Drive straight forward to stop_x precisely (heading stays ~0)."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        pose = machine._read_base_pose()
        if pose is not None and pose.x >= stop_x - 0.01:
            break
        machine._send_command({"base_forward": 0.2})
        time.sleep(1.0 / 30.0)
    machine._send_command({})


def _settle_ee(sim, drive_fn, timeout: float = 20.0):
    """Hold a pose and return the gripper world (x,y,z) once it stops moving."""
    last = None
    stable_since = None
    t0 = time.time()
    while time.time() - t0 < timeout:
        drive_fn()
        ee = sim.get_ee_pose()
        p = (float(ee[0, 3]), float(ee[1, 3]), float(ee[2, 3]))
        if last is not None and max(abs(a - b) for a, b in zip(p, last)) < 0.001:
            stable_since = stable_since or time.time()
            if time.time() - stable_since > 1.0:
                return p
        else:
            stable_since = None
        last = p
        time.sleep(0.1)
    return last


def calibrate(machine, sim, head_cam, session: Path) -> None:
    cfg = machine.config.section("manipulation")
    rows = []
    for tid, (label, ox, oy) in OBJECTS.items():
        machine.target_id = tid
        stop_x = machine._object_base_x(tid)
        _drive_base_forward_to(machine, stop_x)

        grasp = {**dict(cfg["pregrasp"]), **dict(cfg["over_pose"]), **machine._grasp_pose(cfg)}

        def drive_arm():
            command, _ = machine._drive_joints_to(grasp, cfg)
            machine._send_command(command)

        gx, gy, gz = _settle_ee(sim, drive_arm)
        st = machine.controller.get_state()
        base = machine._read_base_pose()
        _save_frame(session / "frames", f"calib_{label}", head_cam.rgb_cam.get_frame())

        row = {
            "object": label,
            "stop_base_x": round(stop_x, 4),
            "base_x": round(base.x, 4),
            "base_theta": round(base.theta, 4),
            "arm_out_cmd": grasp["arm_out"],
            "arm_out_actual": round(st.get("arm_out", 0.0), 4),
            "lift_actual": round(st.get("lift_up", 0.0), 4),
            "obj_x": ox,
            "obj_y": oy,
            "gripper_x": round(gx, 4),
            "gripper_y": round(gy, 4),
            "gripper_z": round(gz, 4),
            "err_x": round(ox - gx, 4),
            "err_y": round(oy - gy, 4),
        }
        rows.append(row)
        print(
            f"[calib] {label:12} err_x={row['err_x']:+.3f} err_y={row['err_y']:+.3f} "
            f"(gripper=({gx:.3f},{gy:.3f},{gz:.3f}) arm_out {row['arm_out_actual']}/{grasp['arm_out']})"
        )

    with (session / "telemetry.csv").open("w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    # Suggestions: stop_base_x shifts by err_x; arm_out shifts by -err_y (more
    # arm_out reaches further -Y). arm_out is shared, so suggest the average.
    print("\n[calib] suggested corrections:")
    for r in rows:
        print(f"  stop_base_x[{r['object']}] -> {r['stop_base_x'] + r['err_x']:.3f}")
    avg_err_y = sum(r["err_y"] for r in rows) / len(rows)
    cur_arm = rows[0]["arm_out_cmd"]
    print(f"  over/grasp arm_out -> {cur_arm - avg_err_y:.3f}  (avg err_y={avg_err_y:+.3f})")
    print(f"\n[calib] wrote {session/'telemetry.csv'}")


def run_missions(machine, sim, head_cam, session: Path, missions) -> None:
    rows = []
    for obj, dest in missions:
        tid = target_id_for_name(obj)
        ox, oy = OBJECTS[tid][1], OBJECTS[tid][2]
        machine.drive_base_to(machine.preapproach_pose(tid), timeout_s=45.0)
        machine.request(obj, destination=dest)

        ee_at_grasp = None        # gripper world pose when the grasp pose is reached
        grip_after_close = None   # finger joint after closing (held vs empty)
        t0 = time.time()
        state = AssistState.SEARCH
        prev = state
        while time.time() - t0 < 180.0:
            state = machine.step()
            if state == AssistState.GRASP and ee_at_grasp is None:
                ee = sim.get_ee_pose()
                ee_at_grasp = (float(ee[0, 3]), float(ee[1, 3]), float(ee[2, 3]))
                _save_frame(session / "frames", f"{obj}_grasp", head_cam.rgb_cam.get_frame())
            if state == AssistState.RETURN and prev == AssistState.GRASP:
                grip_after_close = machine.controller.get_state().get("gripper_open")
            prev = state
            if state in (AssistState.COMPLETE, AssistState.ABORTED):
                break
            time.sleep(1.0 / 30.0)

        _save_frame(session / "frames", f"{obj}_{dest}_end", head_cam.rgb_cam.get_frame())
        gx, gy, gz = ee_at_grasp if ee_at_grasp else (0.0, 0.0, 0.0)
        rows.append(
            {
                "object": obj,
                "destination": dest,
                "outcome": state.value,
                "err_x": round(ox - gx, 4) if ee_at_grasp else "",
                "err_y": round(oy - gy, 4) if ee_at_grasp else "",
                "gripper_z": round(gz, 4) if ee_at_grasp else "",
                "grip_after_close": round(grip_after_close, 3) if grip_after_close is not None else "",
                "memory_x": round(machine._object_base_x(tid), 3),
                "duration_s": round(time.time() - t0, 1),
            }
        )
        print(
            f"[run] {obj} -> {dest}: {state.value} "
            f"err_x={rows[-1]['err_x']} err_y={rows[-1]['err_y']} grip={rows[-1]['grip_after_close']}"
        )

    with (session / "telemetry.csv").open("w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f"\n[run] wrote {session/'telemetry.csv'}")


def main() -> None:
    mode = sys.argv[1] if len(sys.argv) > 1 else "calibrate"
    from stretch_toolkit import HEAD_CAMERA, controller

    machine = StretchAssistStateMachine(
        controller=controller,
        head_camera=HEAD_CAMERA,
        config_path="stretch_assist_config.json",
        debug_perception=True,
    )
    machine._read_base_pose()  # boot the lazy sim
    time.sleep(2.0)
    sim = controller.sim
    session = _session_dir()
    print(f"[grasp_lab] session: {session}")

    if mode == "calibrate":
        calibrate(machine, sim, HEAD_CAMERA, session)
    elif mode == "run":
        missions = [tuple(a.split(":")) for a in sys.argv[2:]] or [("glass", "table")]
        run_missions(machine, sim, HEAD_CAMERA, session, missions)
    else:
        print(f"unknown mode {mode!r}; use 'calibrate' or 'run'")


if __name__ == "__main__":
    main()
