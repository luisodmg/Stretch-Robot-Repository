"""Stretch Assist autonomous retrieval state machine.

This module keeps the control logic on top of ``stretch_toolkit`` abstractions:
controllers, teleop override, and camera info objects. It avoids direct MuJoCo
calls so the same behavior can run in simulation or on a physical Stretch.
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from enum import Enum
import argparse
import json
import math
import os
from pathlib import Path
import time
from typing import Callable, Mapping

from destinations import (
    DEFAULT_DESTINATION,
    DESTINATION_LABELS,
    destination_id_for_name,
)
from perception import DetectedObject, TARGET_OBJECTS, detect_object, detect_objects, target_id_for_name


DEFAULT_CONFIG: dict = {
    "loop_hz": 30,
    "perception": {
        "aruco_dictionary": "DICT_4X4_50",
        "depth_sample_radius": 3,
    },
    # The objects sit on a table to the robot's right (-Y). Search aims the head
    # at that side and sweeps it to find the marker.
    "search": {
        "center_pan": -1.0,
        "center_tilt": -0.7,
        "sweep_pan": 0.6,
        "head_sweep_hz": 0.1,
        "head_gain": 2.0,
        "timeout_s": 30.0,
    },
    # Approach drives straight forward (no base rotation, so the side grasp stays
    # predictable) and stops by odometry at the x where the target is exactly
    # abeam and within the arm's reach. Vision still runs for the live window and
    # to confirm the target before approaching. stop_base_x is per target id:
    # gripper x = base_x - 0.021, objects are at x = 0.25 / 0.45 / 0.65.
    "approach": {
        "head_pan": -1.5,
        "head_tilt": -0.75,
        "head_gain": 2.0,
        "forward_speed": 0.4,
        "min_forward": 0.16,
        "approach_gain": 2.5,
        "stop_base_x_by_target": {"0": 0.23, "1": 0.38, "2": 0.53},
        "stop_base_x_default": 0.38,
        "base_x_tolerance_m": 0.01,
        # How far before the object the interactive loop parks the base before
        # the final forward APPROACH (so it can come from any direction). Kept
        # generous so the upward-facing marker is not viewed edge-on from too
        # close during SEARCH.
        "preapproach_run_m": 0.4,
        "timeout_s": 40.0,
    },
    # Deterministic, camera-free manipulation. The robot has already centered
    # and approached the object with the head camera; these absolute joint
    # targets drive a reliable scripted pick. Tune them live: the JSON config is
    # hot-reloaded, so you can adjust while the simulator runs.
    "manipulation": {
        "position_gain": 3.0,
        "joint_tolerance_m": 0.02,
        "gripper_tolerance_m": 0.01,
        "phase_timeout_s": 8.0,
        # Phase 1: lift high, arm retracted, gripper open. The finger joint opens
        # to ~0.6; open wide (0.4) so the gripper can surround a wide object like
        # the medicine box, not just the thin glass.
        "pregrasp": {
            "lift_up": 0.66,
            "arm_out": 0.0,
            "wrist_yaw_counterclockwise": 0.0,
            "gripper_open": 0.4,
        },
        # Phase 2: still high, extend the arm so the open gripper is OVER the object.
        "over_pose": {
            "arm_out": 0.31,
        },
        # Phase 3: lower the gripper down around the object (no sideways ramming).
        "grasp_pose": {
            "lift_up": 0.5,
            "arm_out": 0.31,
        },
        # Per-object grasp height (ArUco id -> lift). Shorter objects need a
        # LOWER grip so the fingers close around the body, not above it. Glass
        # (tall) 0.45; medicine box (shorter) 0.43; tissue (flat) 0.41.
        "grasp_lift_by_target": {
            "0": 0.45,
            "1": 0.45,
            "2": 0.45,
        },
        "gripper_close_speed": -1.0,
        # Close the gripper for at least min, and keep closing until the fingers
        # stop moving (settled on the object) or max is reached, BEFORE lifting.
        "gripper_close_min_s": 0.6,
        "gripper_close_max_s": 2.5,
        "gripper_close_time_s": 1.6,
        # Grasp-success check after closing: held fingers sit ~ -0.09..+0.09, an
        # empty close goes to ~ -0.24, so anything at/below this is "missed" and
        # the pick is retried (reopen + re-align) up to grasp_max_attempts.
        "grasp_closed_threshold": -0.18,
        "grasp_max_attempts": 3,
        "lift_clearance_m": 0.18,
        # After lifting, retract the arm to this extension so the object rides
        # close to the base and stays put through turns during transport.
        "carry_arm_out": 0.0,
    },
    # RETURN now carries the grasped object to a chosen drop-off point, then
    # RELEASE opens the gripper. Destinations are offsets (dx, dy, dtheta) from
    # the robot's start pose; "user" (0,0,0) goes back to where it began. The
    # base is differential drive, so the controller turns to face the goal and
    # drives forward, reversing when the goal is straight behind (which avoids a
    # 180-degree spin on the way home).
    "return": {
        "distance_tolerance_m": 0.08,
        "heading_tolerance_rad": 0.15,
        "forward_gain": 1.6,
        "turn_gain": 1.2,
        "max_forward": 0.6,
        "max_turn": 0.6,
        "reverse_bearing_rad": 2.356,
        "default_destination": "person",
        # Delivery stations sit along the -Y wall; the robot drives forward
        # (facing +x) to each x and places the object on its -Y side, reusing the
        # pickup arm geometry. Offsets are [dx, dy, dtheta] from the start pose.
        "destinations": {
            "table": [1.30, 0.0, 0.0],
            "shelf": [1.90, 0.0, 0.0],
            "person": [2.50, 0.0, 0.0],
        },
    },
    "release": {
        "open_speed": 1.0,
        "open_time_s": 1.1,
    },
}


# Sign mapping a positive normalized velocity command to an INCREASE in the
# measured joint position (get_state). The simulator's joint_max_speeds are
# negative for the head and wrist joints, so a positive command there actually
# decreases the position. Closed-loop position control must flip those joints;
# lift/arm/gripper already map positively.
_POSITION_COMMAND_SIGN: dict[str, float] = {
    "head_pan_counterclockwise": -1.0,
    "head_tilt_up": -1.0,
    "wrist_yaw_counterclockwise": -1.0,
    "wrist_pitch_up": -1.0,
    "wrist_roll_counterclockwise": -1.0,
    "lift_up": 1.0,
    "arm_out": 1.0,
    "gripper_open": 1.0,
}


# The simulator maps a positive "base_counterclockwise" velocity to a CLOCKWISE
# base rotation (decreasing base_theta), the opposite of its name. The base
# navigation controller works in standard math convention (+ = counterclockwise),
# so its turn commands are multiplied by this sign before being sent. Forward is
# already correct (+ drives toward +x in the robot frame).
_BASE_TURN_COMMAND_SIGN = -1.0


class AssistState(str, Enum):
    IDLE = "IDLE"
    SEARCH = "SEARCH"
    APPROACH = "APPROACH"
    ALIGN = "ALIGN"
    GRASP = "GRASP"
    RETURN = "RETURN"
    PLACE = "PLACE"
    RELEASE = "RELEASE"
    COMPLETE = "COMPLETE"
    ABORTED = "ABORTED"


@dataclass(frozen=True)
class BasePose:
    x: float
    y: float
    theta: float


class HotReloadJsonConfig:
    """JSON config loader that merges user config over defaults when modified."""

    def __init__(self, path: str | Path, defaults: Mapping | None = None):
        self.path = Path(path)
        self.defaults = deepcopy(defaults or DEFAULT_CONFIG)
        self.data = deepcopy(self.defaults)
        self._mtime: float | None = None
        self.reload(force=True)

    def reload(self, *, force: bool = False) -> bool:
        if not self.path.exists():
            self.path.write_text(json.dumps(self.defaults, indent=2) + "\n")

        mtime = self.path.stat().st_mtime
        if not force and self._mtime == mtime:
            return False

        with self.path.open("r", encoding="utf-8") as handle:
            loaded = json.load(handle)

        self.data = _deep_merge(deepcopy(self.defaults), loaded)
        self._mtime = mtime
        return True

    def section(self, name: str) -> dict:
        return self.data.get(name, {})

    def get(self, name: str, default=None):
        return self.data.get(name, default)


class StretchAssistStateMachine:
    """End-to-end object retrieval controller for Stretch Assist."""

    def __init__(
        self,
        *,
        controller,
        teleop=None,
        head_camera=None,
        wrist_camera=None,
        config_path: str | Path = "stretch_assist_config.json",
        detector: Callable[..., DetectedObject | None] = detect_object,
        debug_perception: bool = False,
        feedback: Callable[[str, str | None], None] | None = None,
        vision=None,
    ):
        self.controller = controller
        self.teleop = teleop
        self.head_camera = head_camera
        self.wrist_camera = wrist_camera
        self.config = HotReloadJsonConfig(config_path)
        self.detector = detector
        self.debug_perception = debug_perception
        self._last_perception_debug_at = 0.0
        self.feedback = feedback or self._print_feedback
        self.vision = vision

        self.state = AssistState.IDLE
        self.target_id: int | None = None
        self.target_name: str | None = None
        self.destination_name: str = DEFAULT_DESTINATION
        self.start_pose: BasePose | None = None
        # Last known base-x at which each object can be grasped (it sits on the
        # -Y side there). Seeded from the pickup table, updated after each
        # delivery so the robot remembers where it left things.
        self.object_x: dict[int, float] = {}
        self.last_detection: DetectedObject | None = None
        self.last_message: str | None = None
        self.state_entered_at = time.monotonic()
        self.last_command: dict[str, float] = {}

        # Scripted-manipulation sub-phase trackers.
        self._manip_phase = 0
        self._grasp_phase = 0
        self._place_phase = 0
        self._grasp_lift_target: float | None = None
        # Gripper-closure tracking, so the lift waits until the fingers have
        # actually finished closing on the object (not a fixed time).
        self._grasp_prev_grip: float | None = None
        self._grasp_grip_stable_since = 0.0
        self._grasp_attempts = 0

    def request(self, target: str | int, destination: str | None = None) -> None:
        """Start a retrieval request for a target object and drop-off point."""

        self.target_id = target_id_for_name(target)
        self.target_name = TARGET_OBJECTS[self.target_id]
        if destination is not None:
            self.set_destination(destination)
        # The home reference (for destination offsets) is captured once and kept
        # across interactive missions, so the robot does not have to return to
        # the very start between requests.
        if self.start_pose is None:
            self.start_pose = self._read_base_pose()
        self.last_detection = None
        self._manip_phase = 0
        self._grasp_phase = 0
        self._place_phase = 0
        self._grasp_lift_target = None
        self._grasp_prev_grip = None
        self._grasp_grip_stable_since = 0.0
        self._grasp_attempts = 0
        if self.vision is not None:
            self.vision.target_id = self.target_id
        self.feedback("Destination", DESTINATION_LABELS.get(self.destination_name, self.destination_name))
        self._transition(AssistState.SEARCH, f"looking for {self.target_name}")

    def set_destination(self, destination: str) -> None:
        """Choose the drop-off point the object is carried to after grasping."""

        self.destination_name = destination_id_for_name(destination)

    def step(self, now: float | None = None) -> AssistState:
        """Run one 30 Hz control step and return the current state."""

        now = time.monotonic() if now is None else now
        self.config.reload()

        if self.state in {AssistState.IDLE, AssistState.COMPLETE, AssistState.ABORTED}:
            self._send_command({})
            return self.state

        if self.target_id is None:
            self.abort("no target selected")
            return self.state

        if self.state == AssistState.SEARCH:
            command = self._step_search(now)
        elif self.state == AssistState.APPROACH:
            command = self._step_approach(now)
        elif self.state == AssistState.ALIGN:
            command = self._step_align(now)
        elif self.state == AssistState.GRASP:
            command = self._step_grasp(now)
        elif self.state == AssistState.RETURN:
            command = self._step_return(now)
        elif self.state == AssistState.PLACE:
            command = self._step_place(now)
        elif self.state == AssistState.RELEASE:
            command = self._step_release(now)
        else:
            command = {}

        self._send_command(command)
        return self.state

    def run(
        self, *, max_runtime_s: float | None = None, close_vision: bool = True
    ) -> AssistState:
        """Run until COMPLETE, ABORTED, or an optional timeout.

        Set ``close_vision=False`` to keep the live window open across several
        missions (used by the interactive loop).
        """

        start = time.monotonic()
        try:
            while self.state not in {AssistState.COMPLETE, AssistState.ABORTED}:
                if max_runtime_s is not None and time.monotonic() - start > max_runtime_s:
                    self.abort("runtime limit reached")
                    break

                self.step()
                self._render_vision()
                time.sleep(1.0 / float(self.config.get("loop_hz", 30)))
        except KeyboardInterrupt:
            self._transition(AssistState.ABORTED, "interrupted")
        except ConnectionError:
            # The simulator was stopped (e.g. window closed) mid-loop. Treat as
            # a clean shutdown rather than surfacing a traceback.
            self._transition(AssistState.ABORTED, "simulator disconnected")
        finally:
            self._send_command({}, ignore_connection_error=True)
            if close_vision and self.vision is not None:
                self.vision.close()
        return self.state

    def drive_base_to(
        self, pose: "BasePose", *, timeout_s: float = 40.0
    ) -> bool:
        """Drive the base to ``pose`` and return whether it arrived.

        Used between interactive missions to bring the robot back to its home
        search position so the next pickup starts from the same place.
        """

        start = time.monotonic()
        while time.monotonic() - start < timeout_s:
            self.config.reload()
            try:
                command, reached = self._command_to_pose(pose)
            except ConnectionError:
                return False
            if reached:
                self._send_command({}, ignore_connection_error=True)
                return True
            self._send_command(command, ignore_connection_error=True)
            self._render_vision()
            time.sleep(1.0 / float(self.config.get("loop_hz", 30)))
        return False

    def _render_vision(self) -> None:
        if self.vision is None:
            return
        try:
            still_open = self.vision.render(state=self.state.value, message=self.last_message)
        except Exception:
            return
        if not still_open and self.state not in {AssistState.COMPLETE, AssistState.ABORTED}:
            self.abort("vision window closed")

    def abort(self, reason: str) -> None:
        self._transition(AssistState.ABORTED, reason)
        self._send_command({}, ignore_connection_error=True)

    def _step_search(self, now: float) -> dict[str, float]:
        cfg = self.config.section("search")
        detection, _frame = self._detect_from_camera(self.head_camera)
        if detection is not None:
            self.last_detection = detection
            self._transition(AssistState.APPROACH, f"found {detection.label}")
            return {}

        if self._elapsed(now) > float(cfg["timeout_s"]):
            self.abort("object not found")
            return {}

        # Aim the head at the grasp-side table and sweep the pan to scan it.
        sweep = math.sin(self._elapsed(now) * float(cfg["head_sweep_hz"]) * math.tau)
        targets = {
            "head_pan_counterclockwise": float(cfg["center_pan"]) + sweep * float(cfg["sweep_pan"]),
            "head_tilt_up": float(cfg["center_tilt"]),
        }
        command, _ = self._drive_joints_to(
            targets, {"position_gain": float(cfg["head_gain"]), "joint_tolerance_m": 0.05}
        )
        return command

    def _step_approach(self, now: float) -> dict[str, float]:
        cfg = self.config.section("approach")

        # Keep the head aimed at the grasp-side table so the live vision window
        # keeps showing the target (perception runs purely for display here).
        head_targets = {
            "head_pan_counterclockwise": float(cfg["head_pan"]),
            "head_tilt_up": float(cfg["head_tilt"]),
        }
        head_cmd, _ = self._drive_joints_to(
            head_targets, {"position_gain": float(cfg["head_gain"]), "joint_tolerance_m": 0.05}
        )
        detection, frame = self._detect_from_camera(self.head_camera)
        if detection is not None:
            self.last_detection = detection

        pose = self._read_base_pose()
        stop_x = self._stop_base_x(cfg)
        if self.debug_perception:
            depth_m = detection.depth_m if detection is not None else None
            base_x = pose.x if pose is not None else 0.0
            self._debug_approach(now, depth_m, base_x, stop_x)

        # Stop by odometry at the x where the target is abeam and reachable. This
        # is precise, unlike vision-only centering, so the side grasp lines up.
        if pose is not None and pose.x >= stop_x - float(cfg["base_x_tolerance_m"]):
            self._transition(AssistState.ALIGN, "object beside robot, ready to grasp")
            return {}

        if self._elapsed(now) > float(cfg["timeout_s"]):
            self._transition(AssistState.ALIGN, "approach timed out, proceeding to grasp")
            return {}

        # Drive straight forward, slowing down near the stop point.
        remaining = stop_x - (pose.x if pose is not None else 0.0)
        forward = max(
            float(cfg["min_forward"]),
            min(float(cfg["forward_speed"]), remaining * float(cfg["approach_gain"])),
        )
        command = dict(head_cmd)
        command["base_forward"] = forward
        return command

    def _stop_base_x(self, cfg: Mapping) -> float:
        if self.target_id is not None:
            return self._object_base_x(self.target_id)
        return float(cfg.get("stop_base_x_default", 0.47))

    def _object_base_x(self, target_id: int) -> float:
        """Base-x where ``target_id`` is currently grasped.

        Uses the remembered location if we have one (e.g. after delivering it to
        a station), otherwise the object's pickup-table position from config.
        """

        if target_id in self.object_x:
            return self.object_x[target_id]
        cfg = self.config.section("approach")
        by_target = cfg.get("stop_base_x_by_target", {})
        if str(target_id) in by_target:
            return float(by_target[str(target_id)])
        return float(cfg.get("stop_base_x_default", 0.47))

    def preapproach_pose(self, target_id: int) -> BasePose:
        """Base pose just before ``target_id`` so APPROACH can drive the last bit.

        Lets the interactive loop send the robot straight to where the object is
        (remembered location) from wherever it currently is, instead of going
        back to the start.
        """

        run = float(self.config.section("approach").get("preapproach_run_m", 0.25))
        return BasePose(self._object_base_x(target_id) - run, 0.0, 0.0)

    def _grasp_pose(self, cfg: Mapping) -> dict[str, float]:
        """Grasp pose with a per-object lift height.

        Objects have different heights, so a single lift grabs the tall glass
        near its top (unstable) while suiting the flat tissue. ``grasp_lift_by_
        target`` overrides the lift per ArUco id so each object is gripped around
        its middle.
        """

        pose = dict(cfg["grasp_pose"])
        by_target = cfg.get("grasp_lift_by_target", {})
        if self.target_id is not None and str(self.target_id) in by_target:
            pose["lift_up"] = float(by_target[str(self.target_id)])
        return pose

    def _step_align(self, now: float) -> dict[str, float]:
        """Drive the arm to the grasp pose with closed-loop joint control.

        Deterministic and camera-free (the head camera already approached the
        object). To avoid ramming a light object sideways, the gripper goes:
        1) high + retracted + open, 2) high + extended (over the object), then
        3) descends around it. GRASP then closes and lifts.
        """

        cfg = self.config.section("manipulation")
        pre = dict(cfg["pregrasp"])
        phases = [
            pre,                                            # high, retracted, open
            {**pre, **dict(cfg["over_pose"])},              # high, extended over object
            {**pre, **dict(cfg["over_pose"]), **self._grasp_pose(cfg)},  # descend around it
        ]
        phase = min(self._manip_phase, len(phases) - 1)
        command, reached = self._drive_joints_to(phases[phase], cfg)

        if reached or self._elapsed(now) > float(cfg["phase_timeout_s"]):
            self._manip_phase += 1
            self.state_entered_at = now  # restart the per-phase timeout
            if self._manip_phase >= len(phases):
                self._manip_phase = 0
                self._grasp_phase = 0
                self._grasp_lift_target = None
                self._transition(AssistState.GRASP, "reached pre-grasp pose")
                return {}
        return command

    def _step_grasp(self, now: float) -> dict[str, float]:
        """Close the gripper, then lift the object clear of the table."""

        cfg = self.config.section("manipulation")

        if self._grasp_phase == 0:
            # Close the gripper and DON'T lift until the fingers have actually
            # finished closing on the object (the finger position stops moving).
            # A fixed close time let the lift start mid-close -> crooked grasps.
            state = self._safe_state()
            grip = float(state.get("gripper_open", 0.0)) if state else 0.0
            elapsed = self._elapsed(now)
            if self._grasp_prev_grip is None or abs(grip - self._grasp_prev_grip) > 0.004:
                self._grasp_grip_stable_since = now  # still moving
            self._grasp_prev_grip = grip

            min_s = float(cfg.get("gripper_close_min_s", 0.6))
            max_s = float(cfg.get("gripper_close_max_s", cfg.get("gripper_close_time_s", 2.5)))
            settled = (now - self._grasp_grip_stable_since) >= 0.3
            if elapsed < max_s and not (elapsed >= min_s and settled):
                return {"gripper_open": float(cfg["gripper_close_speed"])}

            # Fully closed -> check the finger position to tell a real grasp from
            # closing on nothing. A held object keeps the fingers open (grip
            # ~ -0.09..+0.09); an empty close goes far more negative (~ -0.24).
            # The narrow box pick is a touch flaky, so on a miss reopen and try
            # the whole align+grasp again instead of carrying air.
            threshold = float(cfg.get("grasp_closed_threshold", -0.18))
            if grip <= threshold:
                self._grasp_attempts += 1
                max_attempts = int(cfg.get("grasp_max_attempts", 3))
                if self._grasp_attempts >= max_attempts:
                    self.abort("could not grasp the object")
                    return {}
                self._manip_phase = 0
                self._grasp_phase = 0
                self._grasp_lift_target = None
                self._grasp_prev_grip = None
                self._grasp_grip_stable_since = now
                self._transition(
                    AssistState.ALIGN,
                    f"grasp missed (grip {grip:.2f}), retry {self._grasp_attempts}/{max_attempts}",
                )
                return {"gripper_open": float(cfg["pregrasp"]["gripper_open"])}

            # Held -> now it is safe to lift.
            self._grasp_phase = 1
            self.state_entered_at = now
            current_lift = float(state.get("lift_up", 0.58)) if state else 0.58
            self._grasp_lift_target = current_lift + float(cfg["lift_clearance_m"])
            return {"gripper_open": float(cfg["gripper_close_speed"])}

        if self._grasp_phase == 1:
            # phase 1: raise the lift to clear the table, holding the gripper shut.
            command, reached = self._drive_joints_to({"lift_up": self._grasp_lift_target}, cfg)
            command["gripper_open"] = float(cfg["gripper_close_speed"])
            if reached or self._elapsed(now) > float(cfg["phase_timeout_s"]):
                self._grasp_phase = 2
                self.state_entered_at = now
                return command
            return command

        # phase 2: retract the arm to a tucked carry pose so the object rides
        # close to the base and does not get flung out during base turns. Keep
        # the gripper actively closed the whole time.
        carry = {
            "arm_out": float(cfg.get("carry_arm_out", 0.0)),
            "lift_up": self._grasp_lift_target,
        }
        command, reached = self._drive_joints_to(carry, cfg)
        command["gripper_open"] = float(cfg["gripper_close_speed"])
        if reached or self._elapsed(now) > float(cfg["phase_timeout_s"]):
            self._transition(AssistState.RETURN, "object secured for transport")
            return command
        return command

    def _drive_joints_to(
        self, targets: Mapping[str, float], cfg: Mapping
    ) -> tuple[dict[str, float], bool]:
        """Proportional position control toward absolute joint targets.

        Returns the normalized velocity command and whether every joint is
        within tolerance. Stays within the ``stretch_toolkit`` velocity API so
        it works on the simulator and on hardware.
        """

        state = self._safe_state()
        if state is None:
            return {}, False

        gain = float(cfg.get("position_gain", 3.0))
        tol = float(cfg.get("joint_tolerance_m", 0.02))
        gtol = float(cfg.get("gripper_tolerance_m", 0.01))

        command: dict[str, float] = {}
        reached = True
        for joint, target in targets.items():
            current = state.get(joint)
            if current is None:
                continue
            error = float(target) - float(current)
            joint_tol = gtol if joint == "gripper_open" else tol
            if abs(error) > joint_tol:
                reached = False
            sign = _POSITION_COMMAND_SIGN.get(joint, 1.0)
            command[joint] = _clamp(error * gain * sign, 1.0)
        return command, reached

    def _safe_state(self) -> dict | None:
        try:
            return self.controller.get_state()
        except Exception:
            return None

    def _debug_approach(self, now: float, depth_m, base_x: float, stop_x: float) -> None:
        if now - self._last_perception_debug_at < 1.0:
            return
        self._last_perception_debug_at = now
        pose = self._read_base_pose()
        depth_txt = f"{depth_m:.2f}m" if depth_m is not None else "?"
        extra = ""
        if pose is not None:
            extra = f" base=({pose.x:+.2f},{pose.y:+.2f},{pose.theta:+.2f})"
        print(
            f"[Stretch Assist] approach debug: depth={depth_txt} base_x={base_x:+.3f} "
            f"target_x={stop_x:.3f}{extra}"
        )

    def _step_return(self, now: float) -> dict[str, float]:
        target = self._destination_pose()
        if target is None:
            self._transition(AssistState.RELEASE, "no start pose recorded")
            return {}

        command, reached = self._command_to_pose(target)
        if reached:
            label = DESTINATION_LABELS.get(self.destination_name, self.destination_name)
            self._place_phase = 0
            self._transition(AssistState.PLACE, f"arrived at {label}, placing object")
            return {}
        # Keep the gripper actively closed while driving so the object cannot slip.
        command["gripper_open"] = float(self.config.section("manipulation")["gripper_close_speed"])
        return command

    def _step_place(self, now: float) -> dict[str, float]:
        """Set the carried object down on the destination surface, then release.

        Mirrors the pick: extend the arm out over the receiving surface (on the
        robot's -Y side, like the pickup table), lower onto it, open the gripper,
        then retract clear. Reuses the tuned pickup poses so no new heights need
        tuning.
        """

        cfg = self.config.section("manipulation")
        rel = self.config.section("release")
        high = float(cfg["pregrasp"]["lift_up"])
        extended = float(cfg["over_pose"]["arm_out"])
        low = float(self._grasp_pose(cfg)["lift_up"])  # set down at the grasp height
        retracted = float(cfg.get("carry_arm_out", 0.0))
        hold = {"gripper_open": float(cfg["gripper_close_speed"])}

        if self._place_phase == 0:  # swing the arm out over the surface, staying high
            command, reached = self._drive_joints_to({"lift_up": high, "arm_out": extended}, cfg)
            command.update(hold)
            if reached or self._elapsed(now) > float(cfg["phase_timeout_s"]):
                self._place_phase = 1
                self.state_entered_at = now
            return command

        if self._place_phase == 1:  # lower the object onto the surface
            command, reached = self._drive_joints_to({"lift_up": low, "arm_out": extended}, cfg)
            command.update(hold)
            if reached or self._elapsed(now) > float(cfg["phase_timeout_s"]):
                self._place_phase = 2
                self.state_entered_at = now
            return command

        if self._place_phase == 2:  # open the gripper to release it onto the surface
            if self._elapsed(now) < float(rel["open_time_s"]):
                return {"gripper_open": float(rel["open_speed"])}
            self._place_phase = 3
            self.state_entered_at = now
            return {}

        # phase 3: retract and raise, leaving the object resting on the surface.
        command, reached = self._drive_joints_to({"lift_up": high, "arm_out": retracted}, cfg)
        if reached or self._elapsed(now) > float(cfg["phase_timeout_s"]):
            # Remember where we left it: the object now sits at this station, so
            # a later request to grab it again goes straight here.
            target_pose = self._destination_pose()
            if target_pose is not None and self.target_id is not None:
                self.object_x[self.target_id] = target_pose.x
            self._transition(AssistState.COMPLETE, "object delivered")
            return {}
        return command

    def _step_release(self, now: float) -> dict[str, float]:
        cfg = self.config.section("release")
        if self._elapsed(now) < float(cfg["open_time_s"]):
            return {"gripper_open": float(cfg["open_speed"])}

        self._transition(AssistState.COMPLETE, "delivery complete")
        return {}

    def _detect_from_camera(
        self, camera, camera_name: str = "head"
    ) -> tuple[DetectedObject | None, object | None]:
        if camera is None:
            return None, None

        frame, depth_frame = _camera_frames(camera)
        if frame is None:
            return None, None

        cfg = self.config.section("perception")
        detection = self.detector(
            frame,
            depth_frame,
            camera_info=camera,
            target_id=self.target_id,
            dictionary_name=cfg["aruco_dictionary"],
            depth_sample_radius=int(cfg["depth_sample_radius"]),
        )
        if self.debug_perception and detection is None:
            self._debug_visible_markers(frame, depth_frame, camera, cfg, camera_name)
        return detection, frame

    def _debug_visible_markers(
        self, frame, depth_frame, camera, cfg: Mapping, camera_name: str = "head"
    ) -> None:
        now = time.monotonic()
        if now - self._last_perception_debug_at < 1.0:
            return
        self._last_perception_debug_at = now

        try:
            visible = detect_objects(
                frame,
                depth_frame,
                camera_info=camera,
                dictionary_name=cfg["aruco_dictionary"],
                depth_sample_radius=int(cfg["depth_sample_radius"]),
            )
        except Exception as exc:
            print(f"[Stretch Assist] perception debug: failed: {exc}")
            return

        head_state = self._read_head_pose()
        pose = ""
        if head_state is not None:
            pan, tilt = head_state
            pose = f" head_pan={pan:.2f} head_tilt={tilt:.2f}"
        if visible:
            labels = ", ".join(
                f"id={item.aruco_id} depth={item.depth_m:.2f}m"
                if item.depth_m is not None
                else f"id={item.aruco_id} depth=?"
                for item in visible
            )
        else:
            labels = "none"
        print(
            f"[Stretch Assist] perception debug: camera={camera_name}"
            f"{pose} visible={labels}"
        )

    def _destination_pose(self) -> BasePose | None:
        """Absolute goal pose for the current destination.

        Destinations are stored as ``[dx, dy, dtheta]`` offsets from the robot's
        start pose, so the same numbers work regardless of where the run began.
        """

        if self.start_pose is None:
            return None
        cfg = self.config.section("return")
        offsets = cfg.get("destinations", {})
        offset = offsets.get(self.destination_name, [0.0, 0.0, 0.0])
        dx, dy, dtheta = (float(offset[0]), float(offset[1]), float(offset[2]))

        c, s = math.cos(self.start_pose.theta), math.sin(self.start_pose.theta)
        return BasePose(
            self.start_pose.x + dx * c - dy * s,
            self.start_pose.y + dx * s + dy * c,
            _wrap_angle(self.start_pose.theta + dtheta),
        )

    def _command_to_pose(self, target: BasePose) -> tuple[dict[str, float], bool]:
        """Differential-drive controller that drives the base to ``target``.

        Turns to face the goal and drives forward, choosing reverse when the goal
        is behind so a return-to-start does not spin 180 degrees. Forward speed is
        scaled by how well the base is aligned, so it turns in place first and
        only commits to driving once roughly pointed at the goal.
        """

        cfg = self.config.section("return")
        current = self._read_base_pose()
        if current is None or target is None:
            return {}, True

        dx = target.x - current.x
        dy = target.y - current.y
        distance = math.hypot(dx, dy)

        if distance <= float(cfg["distance_tolerance_m"]):
            heading_error = _wrap_angle(target.theta - current.theta)
            if abs(heading_error) <= float(cfg["heading_tolerance_rad"]):
                return {}, True
            turn = _BASE_TURN_COMMAND_SIGN * heading_error * float(cfg["turn_gain"])
            return {"base_counterclockwise": _clamp(turn, float(cfg["max_turn"]))}, False

        # Only treat the goal as "behind" (drive in reverse) when it is well past
        # abeam; reversing exactly at +/-90 deg makes the controller chatter
        # between forward and reverse and the base never commits to a turn.
        reverse_threshold = float(cfg.get("reverse_bearing_rad", 2.356))
        bearing = _wrap_angle(math.atan2(dy, dx) - current.theta)
        reverse = abs(bearing) > reverse_threshold
        if reverse:
            bearing = _wrap_angle(bearing - math.pi)

        alignment = max(0.0, math.cos(bearing))  # 0 when sideways, 1 when facing
        speed = _clamp(distance * float(cfg["forward_gain"]) * alignment, float(cfg["max_forward"]))
        if reverse:
            speed = -speed
        turn = _BASE_TURN_COMMAND_SIGN * bearing * float(cfg["turn_gain"])
        return {
            "base_forward": speed,
            "base_counterclockwise": _clamp(turn, float(cfg["max_turn"])),
        }, False

    def _read_head_pose(self) -> tuple[float, float] | None:
        try:
            state = self.controller.get_state()
        except Exception:
            return None

        if not all(key in state for key in ("head_pan_counterclockwise", "head_tilt_up")):
            return None
        return (
            float(state["head_pan_counterclockwise"]),
            float(state["head_tilt_up"]),
        )

    def _read_base_pose(self) -> BasePose | None:
        try:
            state = self.controller.get_state()
        except Exception:
            return None

        if not all(key in state for key in ("base_x", "base_y", "base_theta")):
            return None
        return BasePose(float(state["base_x"]), float(state["base_y"]), float(state["base_theta"]))

    def _send_command(
        self,
        command: dict[str, float],
        *,
        ignore_connection_error: bool = False,
    ) -> None:
        if self.teleop is not None and hasattr(self.teleop, "get_manual_override"):
            command = self.teleop.get_manual_override(command)

        self.last_command = command
        try:
            self.controller.set_velocities(command)
        except ConnectionError:
            if ignore_connection_error:
                return
            raise

    def _transition(self, state: AssistState, message: str | None = None) -> None:
        if self.state == state and message is None:
            return
        self.state = state
        self.state_entered_at = time.monotonic()
        self.last_message = message
        self.feedback(state.value, message)

    def _elapsed(self, now: float) -> float:
        return now - self.state_entered_at

    @staticmethod
    def _print_feedback(state: str, message: str | None = None) -> None:
        suffix = f": {message}" if message else ""
        print(f"[Stretch Assist] {state}{suffix}")


def _camera_frames(camera) -> tuple[object | None, object | None]:
    if hasattr(camera, "get_frames"):
        return camera.get_frames()
    if hasattr(camera, "rgb_cam") and hasattr(camera, "depth_cam"):
        return camera.rgb_cam.get_frame(), camera.depth_cam.get_frame()
    if hasattr(camera, "get_frame"):
        return camera.get_frame(), None
    return None, None


def _normalized_image_error(
    detection: DetectedObject, frame
) -> tuple[float, float]:
    if frame is None:
        return 0.0, 0.0
    height, width = frame.shape[:2]
    if width == 0 or height == 0:
        return 0.0, 0.0

    err_x_px, err_y_px = _pixel_error(detection, frame)
    return err_x_px / (width / 2.0), err_y_px / (height / 2.0)


def _pixel_error(detection: DetectedObject, frame) -> tuple[float, float]:
    height, width = frame.shape[:2]
    return detection.centroid_px[0] - width / 2.0, detection.centroid_px[1] - height / 2.0


def _clamp(value: float, max_abs: float) -> float:
    return max(-max_abs, min(max_abs, value))


def _wrap_angle(angle: float) -> float:
    return (angle + math.pi) % math.tau - math.pi


def _deep_merge(base: dict, override: Mapping) -> dict:
    for key, value in override.items():
        if isinstance(value, Mapping) and isinstance(base.get(key), dict):
            base[key] = _deep_merge(base[key], value)
        else:
            base[key] = value
    return base


def run_stretch_assist(
    target: str | int | None = None,
    *,
    destination: str | None = None,
    interactive: bool = False,
    config_path: str | Path | None = None,
    use_teleop: bool = True,
    debug_perception: bool = False,
    show_vision: bool = True,
    show_wrist: bool = False,
    quiet_sim: bool = True,
    max_runtime_s: float | None = None,
    record_path: str | None = None,
):
    """Launch Stretch Assist using the active ``stretch_toolkit`` backend."""

    import os

    # Lower the simulator camera/viewer render rate and silence the recurring
    # "below requested FPS" warning before the lazy controller spins up the
    # MuJoCo process. Both env vars are read inside the simulator subprocess.
    if quiet_sim:
        if not os.getenv("STRETCH_SIM_CAMERA_HZ"):
            os.environ["STRETCH_SIM_CAMERA_HZ"] = "12"
        os.environ.setdefault("STRETCH_SIM_QUIET", "1")

    from accessible_ui import AccessibleCommandInterface, FeedbackChannel
    from stretch_toolkit import BACKEND_NAME, HEAD_CAMERA, WRIST_CAMERA, controller, teleop

    feedback = FeedbackChannel()
    feedback.announce("Backend", BACKEND_NAME)

    vision = None
    if show_vision:
        try:
            from vision_window import VisionWindow

            # Default to the head camera only: it is where the ArUco markers are
            # visible, and this experimental-GL machine cannot reliably render
            # the head + wrist offscreen cameras plus the viewer at once. The
            # wrist (D405) view can be opted in with show_wrist=True.
            vision = VisionWindow(
                head_camera=HEAD_CAMERA,
                wrist_camera=WRIST_CAMERA if show_wrist else None,
                record_path=record_path,
            )
            if record_path:
                feedback.announce("Recording", record_path)
        except Exception as exc:
            print(f"[Stretch Assist] vision window unavailable: {exc}")
    elif record_path:
        print("[Stretch Assist] --record needs the vision window; ignoring (do not pass --no-vision).")

    machine = StretchAssistStateMachine(
        controller=controller,
        teleop=teleop if use_teleop else None,
        head_camera=HEAD_CAMERA,
        wrist_camera=WRIST_CAMERA,
        config_path=config_path or Path(__file__).with_name("stretch_assist_config.json"),
        feedback=feedback,
        debug_perception=debug_perception,
        vision=vision,
    )

    # Boot the simulator (and its MuJoCo window) BEFORE showing the selector, so
    # the menu appears over a running sim instead of before it. The controller is
    # lazily created, so touching it here forces start-up now.
    feedback.announce("Simulator", "starting up...")
    machine._read_base_pose()

    interface = AccessibleCommandInterface(feedback=feedback)

    try:
        if interactive:
            return _run_interactive(machine, interface, feedback, max_runtime_s=max_runtime_s)

        if target is None:
            selection = interface.wait_for_target()
            target = selection.aruco_id
            # Only prompt for a destination when the object was chosen
            # interactively and one was not already passed in.
            if destination is None:
                destination = interface.wait_for_destination()

        machine.request(target, destination=destination)
        return machine.run(max_runtime_s=max_runtime_s)
    except KeyboardInterrupt:
        feedback.announce("Stretch Assist", "interrupted")
        return machine.state
    finally:
        # Stop the simulator process so the program exits cleanly (Ctrl-C or the
        # window's X) instead of hanging on the sim's non-daemon threads.
        _shutdown_sim(feedback)


def _shutdown_sim(feedback=None) -> None:
    try:
        import stretch_toolkit

        sim = getattr(stretch_toolkit, "_sim", None)
        if sim is not None:
            if feedback is not None:
                feedback.announce("Simulator", "shutting down")
            # Stop the MuJoCo subprocess directly. sim.stop() also tries to join
            # background threads with a 10s timeout each (~30s of hanging), which
            # is what made the app feel like it never closed.
            sim.stop_mujoco_process()
    except Exception:
        pass


def _run_interactive(machine, interface, feedback, *, max_runtime_s=None):
    """Keep accepting "grab object X, drop at Y" commands until the user quits.

    Between missions the robot drives back to its home pickup position so each
    new request starts from the same place. Closing the selector window (or
    Ctrl-C) ends the session.
    """

    feedback.announce(
        "Interactive", "pick an object, then a destination; close the selector or press Ctrl-C to quit"
    )

    try:
        while True:
            try:
                selection = interface.wait_for_target()
                destination = interface.wait_for_destination()
            except (RuntimeError, EOFError, KeyboardInterrupt):
                break

            # Go straight to where the object was last seen (no trip back to the
            # start), then run the pickup + delivery.
            feedback.announce("Heading out", f"going to fetch {selection.label}")
            machine.drive_base_to(machine.preapproach_pose(selection.aruco_id))
            machine.request(selection.aruco_id, destination=destination)
            machine.run(max_runtime_s=max_runtime_s, close_vision=False)

            # Quit the session if the simulator was closed mid-mission (its X) or
            # the live window was closed.
            message = machine.last_message or ""
            if machine.state == AssistState.ABORTED and "disconnect" in message:
                break
            if machine.vision is not None and getattr(machine.vision, "_closed", False):
                break
    finally:
        if machine.vision is not None:
            machine.vision.close()

    feedback.announce("Interactive", "session ended")
    return machine.state


def main() -> None:
    parser = argparse.ArgumentParser(description="Run Stretch Assist autonomous retrieval.")
    parser.add_argument(
        "--target",
        choices=list(TARGET_OBJECTS.values()) + [str(item) for item in TARGET_OBJECTS],
        help="Target object to retrieve. If omitted, opens the accessible selector.",
    )
    parser.add_argument(
        "--destination",
        choices=list(DESTINATION_LABELS),
        help="Where to carry the object after grasping. If omitted, the selector "
        "asks (or defaults to 'person').",
    )
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="Keep asking for object + destination and run repeated deliveries, "
        "homing between missions. Close the selector or press Ctrl-C to quit.",
    )
    parser.add_argument(
        "--config",
        default=str(Path(__file__).with_name("stretch_assist_config.json")),
        help="Path to hot-reloadable Stretch Assist JSON config.",
    )
    parser.add_argument(
        "--no-teleop",
        action="store_true",
        help="Disable keyboard/gamepad override for autonomous simulator testing.",
    )
    parser.add_argument(
        "--debug-perception",
        action="store_true",
        help="Print visible ArUco IDs during search for simulator debugging.",
    )
    parser.add_argument(
        "--no-vision",
        action="store_true",
        help="Disable the live robot-vision window.",
    )
    parser.add_argument(
        "--show-wrist",
        action="store_true",
        help="Also show the wrist (D405) camera panel. May be unreliable on "
        "machines that cannot render multiple offscreen cameras at once.",
    )
    parser.add_argument(
        "--loud-sim",
        action="store_true",
        help="Keep the default 30 FPS render rate (and its FPS warning spam).",
    )
    parser.add_argument(
        "--max-runtime",
        type=float,
        default=None,
        help="Abort automatically after this many seconds (useful for demos/tests).",
    )
    parser.add_argument(
        "--record",
        nargs="?",
        const="__auto__",
        default=None,
        metavar="PATH",
        help="Record the robot-vision window to an mp4 for demos. Pass a path, or "
        "just --record to auto-name it under recordings/. Needs the vision window.",
    )
    args = parser.parse_args()

    record_path = args.record
    if record_path == "__auto__":
        import datetime as _dt

        stamp = _dt.datetime.now().strftime("%Y%m%d-%H%M%S")
        record_path = str(Path(__file__).with_name("recordings") / f"stretch_assist_{stamp}.mp4")
    run_stretch_assist(
        args.target,
        destination=args.destination,
        interactive=args.interactive,
        config_path=args.config,
        use_teleop=not args.no_teleop,
        debug_perception=args.debug_perception,
        show_vision=not args.no_vision,
        show_wrist=args.show_wrist,
        quiet_sim=not args.loud_sim,
        max_runtime_s=args.max_runtime,
        record_path=record_path,
    )
    # The toolkit's gamepad listener is a non-daemon thread that keeps the
    # process alive on a normal return. The sim subprocess was already stopped,
    # so force an immediate clean exit.
    os._exit(0)


if __name__ == "__main__":
    main()
