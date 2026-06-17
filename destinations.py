"""Drop-off destinations for Stretch Assist delivery navigation.

After grasping, the robot carries the object to one of these named stations and
places it on top of the station's surface. Only the labels, order, and default
live here; the actual target coordinates (an offset from the robot's start pose)
are in the hot-reloadable JSON config under ``return.destinations`` so they can
be tuned without code changes. The matching furniture is drawn in
``stretch_mujoco/models/scene.xml``.
"""

from __future__ import annotations


DESTINATION_LABELS: dict[str, str] = {
    "table": "Table",
    "shelf": "Shelf",
    "person": "Person",
}

# Order shown in the selector / CLI help.
DESTINATION_ORDER: list[str] = ["table", "shelf", "person"]

DEFAULT_DESTINATION = "person"


def destination_id_for_name(name: str) -> str:
    """Return the canonical destination key for a name or label."""

    key = str(name).strip().lower().replace(" ", "_").replace("-", "_")
    if key in DESTINATION_LABELS:
        return key
    for canonical, label in DESTINATION_LABELS.items():
        if key == label.lower().replace(" ", "_"):
            return canonical

    valid = ", ".join(DESTINATION_LABELS)
    raise ValueError(f"Unknown destination '{name}'. Expected one of: {valid}")
