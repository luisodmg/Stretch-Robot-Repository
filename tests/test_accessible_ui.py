import pytest

from accessible_ui import AccessibleCommandInterface, FeedbackChannel, TargetSelection


def test_selection_builds_accessible_target_record():
    ui = AccessibleCommandInterface(force_console=True)

    selection = ui._selection(0, "test")

    assert isinstance(selection, TargetSelection)
    assert selection.name == "medicine_box"
    assert selection.aruco_id == 0
    assert selection.label == "Medicine box"
    assert selection.source == "test"


def test_feedback_channel_records_and_emits_status():
    emitted = []
    feedback = FeedbackChannel(writer=emitted.append)

    feedback.announce("SEARCH", "looking for glass")

    assert feedback.last_state == "SEARCH"
    assert emitted == ["[Stretch Assist] SEARCH: looking for glass"]


def test_console_selection_rejects_unknown_target(monkeypatch):
    ui = AccessibleCommandInterface(force_console=True)
    inputs = iter(["unknown", "glass"])
    monkeypatch.setattr("builtins.input", lambda _prompt: next(inputs))

    selection = ui.wait_for_target()

    assert selection.name == "glass"
    assert selection.aruco_id == 1
