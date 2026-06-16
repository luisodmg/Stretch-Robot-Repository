from pathlib import Path

import numpy as np

from perception import DetectedObject
from state_machine import AssistState, StretchAssistStateMachine


class FakeController:
    def __init__(self):
        self.commands = []
        self.state = {
            "base_x": 0.0,
            "base_y": 0.0,
            "base_theta": 0.0,
            "gripper_open": 0.0,
            "head_pan_counterclockwise": 0.0,
            "head_tilt_up": 0.0,
            "lift_up": 0.0,
            "arm_out": 0.0,
            "wrist_yaw_counterclockwise": 0.0,
        }

    def set_velocities(self, command):
        self.commands.append(command)

    def get_state(self):
        return dict(self.state)


class FakeTeleop:
    def get_manual_override(self, command):
        merged = dict(command)
        merged["manual_checked"] = 0.0
        return merged


class FakeCamera:
    def __init__(self):
        self.frame = np.zeros((100, 100, 3), dtype=np.uint8)
        self.depth = np.ones((100, 100), dtype=float)

    def get_frames(self):
        return self.frame, self.depth


def _detection(depth=0.7, center=(50.0, 50.0)):
    return DetectedObject(
        name="medicine_box",
        aruco_id=0,
        centroid_px=center,
        depth_m=depth,
        position_camera_m=(0.0, 0.0, depth),
        marker_area_px=100.0,
        corners_px=((40.0, 40.0), (60.0, 40.0), (60.0, 60.0), (40.0, 60.0)),
    )


def test_search_transitions_to_approach_and_uses_manual_override(tmp_path: Path):
    controller = FakeController()
    camera = FakeCamera()

    def detector(*args, **kwargs):
        return _detection()

    machine = StretchAssistStateMachine(
        controller=controller,
        teleop=FakeTeleop(),
        head_camera=camera,
        wrist_camera=camera,
        config_path=tmp_path / "assist.json",
        detector=detector,
    )
    machine.request("medicine_box")

    state = machine.step()

    assert state == AssistState.APPROACH
    assert controller.commands[-1] == {"manual_checked": 0.0}


def test_approach_transitions_to_align_when_centered_and_close(tmp_path: Path):
    controller = FakeController()
    camera = FakeCamera()

    # Base has driven past the odometry stop point, so the object is abeam.
    controller.state["base_x"] = 0.6

    def detector(*args, **kwargs):
        return _detection(depth=0.55, center=(50.0, 50.0))

    machine = StretchAssistStateMachine(
        controller=controller,
        head_camera=camera,
        wrist_camera=camera,
        config_path=tmp_path / "assist.json",
        detector=detector,
    )
    machine.request(0)
    machine.state = AssistState.APPROACH

    state = machine.step()

    assert state == AssistState.ALIGN
    assert controller.commands[-1] == {}


def test_approach_drives_forward_until_target_is_abeam(tmp_path: Path):
    controller = FakeController()
    camera = FakeCamera()

    def detector(*args, **kwargs):
        # Marker off-center: the object is ahead, not yet beside the robot.
        return _detection(depth=0.4, center=(80.0, 50.0))

    machine = StretchAssistStateMachine(
        controller=controller,
        head_camera=camera,
        wrist_camera=camera,
        config_path=tmp_path / "assist.json",
        detector=detector,
    )
    machine.request(0)
    machine.state = AssistState.APPROACH

    state = machine.step()

    assert state == AssistState.APPROACH
    assert controller.commands[-1]["base_forward"] > 0.0


def test_search_sweeps_head_tilt_when_target_missing(tmp_path: Path):
    controller = FakeController()
    camera = FakeCamera()

    machine = StretchAssistStateMachine(
        controller=controller,
        head_camera=camera,
        wrist_camera=camera,
        config_path=tmp_path / "assist.json",
        detector=lambda *args, **kwargs: None,
    )
    machine.request("medicine_box")

    state = machine.step(now=machine.state_entered_at)

    assert state == AssistState.SEARCH
    assert "head_tilt_up" in controller.commands[-1]
    assert controller.commands[-1]["head_tilt_up"] != 0.0
    assert controller.commands[-1].get("base_counterclockwise", 0.0) == 0.0
