"""Accessible command and feedback interface for Stretch Assist."""

from __future__ import annotations

from dataclasses import dataclass
import sys
import time
from typing import Callable, Iterable

from destinations import (
    DEFAULT_DESTINATION,
    DESTINATION_LABELS,
    DESTINATION_ORDER,
    destination_id_for_name,
)
from perception import TARGET_LABELS, TARGET_OBJECTS, target_id_for_name


@dataclass(frozen=True)
class TargetSelection:
    """Object request emitted by the accessible interface."""

    name: str
    aruco_id: int
    label: str
    source: str
    timestamp: float


class FeedbackChannel:
    """Small feedback sink used by the state machine and UI."""

    def __init__(self, writer: Callable[[str], None] | None = None):
        self._writer = writer or print
        self.last_state: str | None = None

    def announce(self, state: str, message: str | None = None) -> None:
        self.last_state = state
        suffix = f": {message}" if message else ""
        self._writer(f"[Stretch Assist] {state}{suffix}")

    def __call__(self, state: str, message: str | None = None) -> None:
        self.announce(state, message)


class AccessibleCommandInterface:
    """Large-button target selector with a console fallback.

    The GUI uses Tkinter from the Python standard library, avoiding new runtime
    dependencies while satisfying the accessible large-button requirement.
    """

    def __init__(
        self,
        *,
        targets: Iterable[int] | None = None,
        title: str = "Stretch Assist",
        force_console: bool = False,
        feedback: FeedbackChannel | None = None,
    ):
        self.target_ids = list(targets) if targets is not None else list(TARGET_OBJECTS)
        self.title = title
        self.force_console = force_console
        self.feedback = feedback or FeedbackChannel()

    def wait_for_target(self) -> TargetSelection:
        """Block until the user selects a target object."""

        if not self.force_console:
            try:
                return self._wait_with_tk()
            except Exception as exc:
                self.feedback.announce("Interface", f"using console fallback ({exc})")

        return self._wait_with_console()

    def wait_for_destination(self) -> str:
        """Block until the user picks a drop-off point; returns its key."""

        if not self.force_console:
            try:
                return self._destination_with_tk()
            except Exception as exc:
                self.feedback.announce("Interface", f"using console fallback ({exc})")

        return self._destination_with_console()

    def _destination_with_tk(self) -> str:
        import tkinter as tk
        from tkinter import font

        chosen: dict[str, str] = {}
        root = tk.Tk()
        root.title(self.title)
        root.geometry("640x420")
        root.minsize(480, 320)
        root.configure(bg="#f5f7fb")

        title_font = font.Font(family="Segoe UI", size=28, weight="bold")
        button_font = font.Font(family="Segoe UI", size=22, weight="bold")

        tk.Label(
            root, text="Choose destination", font=title_font, bg="#f5f7fb", fg="#102033"
        ).pack(pady=(22, 16))

        button_frame = tk.Frame(root, bg="#f5f7fb")
        button_frame.pack(fill=tk.BOTH, expand=True, padx=28, pady=18)
        button_frame.columnconfigure(tuple(range(len(DESTINATION_ORDER))), weight=1, uniform="dest")
        button_frame.rowconfigure(0, weight=1)

        palette = [("#2357a6", "#ffffff"), ("#13795b", "#ffffff"), ("#8b5a00", "#ffffff")]

        def choose(name: str) -> None:
            chosen["value"] = name
            root.after(120, root.destroy)

        for idx, name in enumerate(DESTINATION_ORDER):
            bg, fg = palette[idx % len(palette)]
            button = tk.Button(
                button_frame,
                text=DESTINATION_LABELS[name],
                command=lambda value=name: choose(value),
                font=button_font,
                bg=bg,
                fg=fg,
                activebackground=bg,
                activeforeground=fg,
                relief=tk.FLAT,
                bd=0,
                padx=18,
                pady=24,
                wraplength=170,
            )
            button.grid(row=0, column=idx, sticky="nsew", padx=8, pady=8)
            root.bind(str(idx + 1), lambda _event, value=name: choose(value))

        root.bind("<Escape>", lambda _event: root.destroy())
        root.mainloop()

        if "value" not in chosen:
            raise RuntimeError("no destination selected")
        return chosen["value"]

    def _destination_with_console(self) -> str:
        prompt_lines = ["Select a destination:"]
        for idx, name in enumerate(DESTINATION_ORDER, start=1):
            prompt_lines.append(f"{idx}. {DESTINATION_LABELS[name]}")
        prompt = "\n".join(prompt_lines) + f"\n[default: {DESTINATION_LABELS[DEFAULT_DESTINATION]}]\n> "

        while True:
            raw = input(prompt).strip()
            if not raw:
                return DEFAULT_DESTINATION
            try:
                if raw.isdigit() and 1 <= int(raw) <= len(DESTINATION_ORDER):
                    return DESTINATION_ORDER[int(raw) - 1]
                return destination_id_for_name(raw)
            except ValueError as exc:
                print(exc, file=sys.stderr)

    def _selection(self, aruco_id: int, source: str) -> TargetSelection:
        return TargetSelection(
            name=TARGET_OBJECTS[aruco_id],
            aruco_id=aruco_id,
            label=TARGET_LABELS[aruco_id],
            source=source,
            timestamp=time.time(),
        )

    def _wait_with_tk(self) -> TargetSelection:
        import tkinter as tk
        from tkinter import font

        selection: dict[str, TargetSelection] = {}
        root = tk.Tk()
        root.title(self.title)
        root.geometry("640x420")
        root.minsize(480, 320)
        root.configure(bg="#f5f7fb")

        title_font = font.Font(family="Segoe UI", size=28, weight="bold")
        button_font = font.Font(family="Segoe UI", size=24, weight="bold")
        status_font = font.Font(family="Segoe UI", size=14)

        title = tk.Label(
            root,
            text="Choose object",
            font=title_font,
            bg="#f5f7fb",
            fg="#102033",
        )
        title.pack(pady=(22, 10))

        status = tk.Label(
            root,
            text="",
            font=status_font,
            bg="#f5f7fb",
            fg="#3d4b5f",
        )
        status.pack(pady=(0, 12))

        button_frame = tk.Frame(root, bg="#f5f7fb")
        button_frame.pack(fill=tk.BOTH, expand=True, padx=28, pady=18)
        button_frame.columnconfigure(tuple(range(len(self.target_ids))), weight=1, uniform="target")
        button_frame.rowconfigure(0, weight=1)

        colors = {
            0: ("#2357a6", "#ffffff"),
            1: ("#13795b", "#ffffff"),
            2: ("#8b5a00", "#ffffff"),
        }

        def choose(aruco_id: int) -> None:
            status.configure(text=f"Selected: {TARGET_LABELS[aruco_id]}")
            selection["value"] = self._selection(aruco_id, "button_gui")
            root.after(120, root.destroy)

        for idx, aruco_id in enumerate(self.target_ids):
            bg, fg = colors.get(aruco_id, ("#34495e", "#ffffff"))
            button = tk.Button(
                button_frame,
                text=TARGET_LABELS[aruco_id],
                command=lambda value=aruco_id: choose(value),
                font=button_font,
                bg=bg,
                fg=fg,
                activebackground=bg,
                activeforeground=fg,
                relief=tk.FLAT,
                bd=0,
                padx=18,
                pady=24,
                wraplength=170,
            )
            button.grid(row=0, column=idx, sticky="nsew", padx=8, pady=8)
            root.bind(str(idx + 1), lambda _event, value=aruco_id: choose(value))

        root.bind("<Escape>", lambda _event: root.destroy())
        root.mainloop()

        if "value" not in selection:
            raise RuntimeError("no object selected")
        return selection["value"]

    def _wait_with_console(self) -> TargetSelection:
        prompt_lines = ["Select an object:"]
        for idx, aruco_id in enumerate(self.target_ids, start=1):
            prompt_lines.append(f"{idx}. {TARGET_LABELS[aruco_id]}")
        prompt = "\n".join(prompt_lines) + "\n> "

        while True:
            raw = input(prompt).strip()
            try:
                if raw.isdigit() and 1 <= int(raw) <= len(self.target_ids):
                    return self._selection(self.target_ids[int(raw) - 1], "console")

                return self._selection(target_id_for_name(raw), "console")
            except ValueError as exc:
                print(exc, file=sys.stderr)


def choose_target(force_console: bool = False) -> TargetSelection:
    """Convenience helper used by scripts."""

    return AccessibleCommandInterface(force_console=force_console).wait_for_target()


if __name__ == "__main__":
    selected = choose_target()
    print(f"{selected.name} ({selected.aruco_id})")
