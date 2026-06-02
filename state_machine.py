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
from pathlib import Path
import time
from typing import Callable, Mapping

from perception import DetectedObject, TARGET_OBJECTS, detect_object, target_id_for_name


DEFAULT_CONFIG: dict = {
    "loop_hz": 30,
    "perception": {
        "aruco_dictionary": "DICT_4X4_50",
        "depth_sample_radius": 3,
    },
    "search": {
        "base_turn_speed": 0.25,
        "head_pan_speed": 0.18,
        "head_tilt_speed": 1.0,
        "head_sweep_hz": 0.2,
        "timeout_s": 25.0,
    },
    "approach": {
        "desired_distance_m": 0.72,
        "distance_tolerance_m": 0.08,
        "forward_gain": 0.7,
        "turn_gain": 0.8,
        "head_pan_gain": 0.25,
        "max_forward": 0.45,
        "max_turn": 0.35,
    },
    "align": {
        "center_tolerance_px": 26.0,
        "desired_distance_m": 0.34,
        "distance_tolerance_m": 0.06,
        "wrist_yaw_gain": 0.55,
        "wrist_pitch_gain": 0.55,
        "arm_gain": 0.9,
        "max_wrist_speed": 0.45,
        "max_arm_speed": 0.35,
        "timeout_s": 18.0,
    },
    "grasp": {
        "close_speed": -1.0,
        "close_time_s": 1.4,
        "closed_threshold": 0.08,
        "assume_success_without_gripper_state": True,
        "retries": 2,
    },
    "return": {
        "distance_tolerance_m": 0.08,
        "heading_tolerance_rad": 0.12,
        "forward_gain": 0.85,
        "turn_gain": 1.2,
        "max_forward": 0.45,
        "max_turn": 0.45,
    },
    "release": {
        "open_speed": 1.0,
        "open_time_s": 1.1,
    },
}


class AssistState(str, Enum):
    IDLE = "IDLE"
    SEARCH = "SEARCH"
    APPROACH = "APPROACH"
    ALIGN = "ALIGN"
    GRASP = "GRASP"
    RETURN = "RETURN"
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
        feedback: Callable[[str, str | None], None] | None = None,
    ):
        self.controller = controller
        self.teleop = teleop
        self.head_camera = head_camera
        self.wrist_camera = wrist_camera
        self.config = HotReloadJsonConfig(config_path)
        self.detector = detector
        self.feedback = feedback or self._print_feedback

        self.state = AssistState.IDLE
        self.target_id: int | None = None
        self.target_name: str | None = None
        self.start_pose: BasePose | None = None
        self.last_detection: DetectedObject | None = None
        self.grasp_attempts = 0
        self.state_entered_at = time.monotonic()
        self.last_command: dict[str, float] = {}

    def request(self, target: str | int) -> None:
        """Start a retrieval request for a target object."""

        self.target_id = target_id_for_name(target)
        self.target_name = TARGET_OBJECTS[self.target_id]
        self.start_pose = self._read_base_pose()
        self.grasp_attempts = 0
        self.last_detection = None
        self._transition(AssistState.SEARCH, f"looking for {self.target_name}")

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
        elif self.state == AssistState.RELEASE:
            command = self._step_release(now)
        else:
            command = {}

        self._send_command(command)
        return self.state

    def run(self, *, max_runtime_s: float | None = None) -> AssistState:
        """Run until COMPLETE, ABORTED, or an optional timeout."""

        start = time.monotonic()
        try:
            while self.state not in {AssistState.COMPLETE, AssistState.ABORTED}:
                if max_runtime_s is not None and time.monotonic() - start > max_runtime_s:
                    self.abort("runtime limit reached")
                    break

                self.step()
                time.sleep(1.0 / float(self.config.get("loop_hz", 30)))
        except KeyboardInterrupt:
            self.abort("interrupted")
        finally:
            self._send_command({})
        return self.state

    def abort(self, reason: str) -> None:
        self._transition(AssistState.ABORTED, reason)
        self._send_command({})

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

        phase = self._elapsed(now) * float(cfg["head_sweep_hz"]) * math.tau
        return {
            "base_counterclockwise": float(cfg["base_turn_speed"]),
            "head_pan_counterclockwise": math.sin(phase) * float(cfg["head_pan_speed"]),
            "head_tilt_up": math.cos(phase) * float(cfg.get("head_tilt_speed", 0.0)),
        }

    def _step_approach(self, now: float) -> dict[str, float]:
        cfg = self.config.section("approach")
        detection, frame = self._detect_from_camera(self.head_camera)
        if detection is None:
            self._transition(AssistState.SEARCH, "lost target during approach")
            return {}

        self.last_detection = detection
        image_error_x, _image_error_y = _normalized_image_error(detection, frame)
        depth_m = detection.depth_m
        if depth_m is None:
            return {
                "base_counterclockwise": _clamp(
                    image_error_x * float(cfg["turn_gain"]), float(cfg["max_turn"])
                )
            }

        distance_error = depth_m - float(cfg["desired_distance_m"])
        if (
            abs(distance_error) <= float(cfg["distance_tolerance_m"])
            and abs(image_error_x) <= 0.12
        ):
            self._transition(AssistState.ALIGN, "close enough for wrist alignment")
            return {}

        return {
            "base_forward": _clamp(distance_error * float(cfg["forward_gain"]), float(cfg["max_forward"])),
            "base_counterclockwise": _clamp(
                image_error_x * float(cfg["turn_gain"]), float(cfg["max_turn"])
            ),
            "head_pan_counterclockwise": _clamp(
                image_error_x * float(cfg["head_pan_gain"]), float(cfg["max_turn"])
            ),
        }

    def _step_align(self, now: float) -> dict[str, float]:
        cfg = self.config.section("align")
        detection, frame = self._detect_from_camera(self.wrist_camera)
        if detection is None:
            if self._elapsed(now) > float(cfg["timeout_s"]):
                self._transition(AssistState.SEARCH, "wrist camera lost target")
            return {"arm_out": -0.15}

        self.last_detection = detection
        err_x_norm, err_y_norm = _normalized_image_error(detection, frame)
        err_x_px, err_y_px = _pixel_error(detection, frame)
        depth_m = detection.depth_m
        distance_error = None if depth_m is None else depth_m - float(cfg["desired_distance_m"])

        centered = (
            abs(err_x_px) <= float(cfg["center_tolerance_px"])
            and abs(err_y_px) <= float(cfg["center_tolerance_px"])
        )
        depth_ready = distance_error is None or abs(distance_error) <= float(cfg["distance_tolerance_m"])
        if centered and depth_ready:
            self._transition(AssistState.GRASP, "aligned with target")
            return {}

        command = {
            "wrist_yaw_counterclockwise": _clamp(
                err_x_norm * float(cfg["wrist_yaw_gain"]), float(cfg["max_wrist_speed"])
            ),
            "wrist_pitch_up": _clamp(
                err_y_norm * float(cfg["wrist_pitch_gain"]), float(cfg["max_wrist_speed"])
            ),
        }
        if distance_error is not None:
            command["arm_out"] = _clamp(
                distance_error * float(cfg["arm_gain"]), float(cfg["max_arm_speed"])
            )
        return command

    def _step_grasp(self, now: float) -> dict[str, float]:
        cfg = self.config.section("grasp")
        if self._elapsed(now) < float(cfg["close_time_s"]):
            return {"gripper_open": float(cfg["close_speed"])}

        if self._verify_grasp():
            self._transition(AssistState.RETURN, "grasp verified")
            return {}

        self.grasp_attempts += 1
        if self.grasp_attempts > int(cfg["retries"]):
            self.abort("grasp failed")
            return {}

        self._transition(AssistState.ALIGN, f"retrying grasp {self.grasp_attempts}")
        return {"gripper_open": 1.0}

    def _step_return(self, now: float) -> dict[str, float]:
        if self.start_pose is None:
            self._transition(AssistState.RELEASE, "no start pose recorded")
            return {}

        command, reached = self._command_to_start_pose()
        if reached:
            self._transition(AssistState.RELEASE, "back at user position")
            return {}
        return command

    def _step_release(self, now: float) -> dict[str, float]:
        cfg = self.config.section("release")
        if self._elapsed(now) < float(cfg["open_time_s"]):
            return {"gripper_open": float(cfg["open_speed"])}

        self._transition(AssistState.COMPLETE, "delivery complete")
        return {}

    def _detect_from_camera(self, camera) -> tuple[DetectedObject | None, object | None]:
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
        return detection, frame

    def _verify_grasp(self) -> bool:
        cfg = self.config.section("grasp")
        try:
            state = self.controller.get_state()
        except Exception:
            return bool(cfg["assume_success_without_gripper_state"])

        gripper = state.get("gripper_open")
        if gripper is None:
            return bool(cfg["assume_success_without_gripper_state"])
        return float(gripper) <= float(cfg["closed_threshold"])

    def _command_to_start_pose(self) -> tuple[dict[str, float], bool]:
        cfg = self.config.section("return")
        current = self._read_base_pose()
        if current is None or self.start_pose is None:
            return {}, True

        dx = self.start_pose.x - current.x
        dy = self.start_pose.y - current.y
        distance = math.hypot(dx, dy)
        final_heading_error = _wrap_angle(self.start_pose.theta - current.theta)

        if distance <= float(cfg["distance_tolerance_m"]):
            if abs(final_heading_error) <= float(cfg["heading_tolerance_rad"]):
                return {}, True
            return {
                "base_counterclockwise": _clamp(
                    final_heading_error * float(cfg["turn_gain"]), float(cfg["max_turn"])
                )
            }, False

        target_heading = math.atan2(dy, dx)
        heading_error = _wrap_angle(target_heading - current.theta)
        if abs(heading_error) > 0.35:
            return {
                "base_counterclockwise": _clamp(
                    heading_error * float(cfg["turn_gain"]), float(cfg["max_turn"])
                )
            }, False

        return {
            "base_forward": _clamp(distance * float(cfg["forward_gain"]), float(cfg["max_forward"])),
            "base_counterclockwise": _clamp(
                heading_error * float(cfg["turn_gain"]), float(cfg["max_turn"])
            ),
        }, False

    def _read_base_pose(self) -> BasePose | None:
        try:
            state = self.controller.get_state()
        except Exception:
            return None

        if not all(key in state for key in ("base_x", "base_y", "base_theta")):
            return None
        return BasePose(float(state["base_x"]), float(state["base_y"]), float(state["base_theta"]))

    def _send_command(self, command: dict[str, float]) -> None:
        if self.teleop is not None and hasattr(self.teleop, "get_manual_override"):
            command = self.teleop.get_manual_override(command)

        self.last_command = command
        self.controller.set_velocities(command)

    def _transition(self, state: AssistState, message: str | None = None) -> None:
        if self.state == state and message is None:
            return
        self.state = state
        self.state_entered_at = time.monotonic()
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


def run_stretch_assist(target: str | int | None = None, *, config_path: str | Path | None = None):
    """Launch Stretch Assist using the active ``stretch_toolkit`` backend."""

    from accessible_ui import AccessibleCommandInterface, FeedbackChannel
    from stretch_toolkit import BACKEND_NAME, HEAD_CAMERA, WRIST_CAMERA, controller, teleop

    feedback = FeedbackChannel()
    feedback.announce("Backend", BACKEND_NAME)

    if target is None:
        selection = AccessibleCommandInterface(feedback=feedback).wait_for_target()
        target = selection.aruco_id

    machine = StretchAssistStateMachine(
        controller=controller,
        teleop=teleop,
        head_camera=HEAD_CAMERA,
        wrist_camera=WRIST_CAMERA,
        config_path=config_path or Path(__file__).with_name("stretch_assist_config.json"),
        feedback=feedback,
    )
    machine.request(target)
    return machine.run()


def main() -> None:
    parser = argparse.ArgumentParser(description="Run Stretch Assist autonomous retrieval.")
    parser.add_argument(
        "--target",
        choices=list(TARGET_OBJECTS.values()) + [str(item) for item in TARGET_OBJECTS],
        help="Target object to retrieve. If omitted, opens the accessible selector.",
    )
    parser.add_argument(
        "--config",
        default=str(Path(__file__).with_name("stretch_assist_config.json")),
        help="Path to hot-reloadable Stretch Assist JSON config.",
    )
    args = parser.parse_args()
    run_stretch_assist(args.target, config_path=args.config)


if __name__ == "__main__":
    main()
