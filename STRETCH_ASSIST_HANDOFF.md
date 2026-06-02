# Stretch Assist Handoff

Date: 2026-06-01
Branch: `main`
Latest reviewed remote commit: `16288d5 fix: widen stretch assist head search`

## Objective

Implement and test a Stretch Assist workflow in the MuJoCo simulator:

1. Detect objects using ArUco markers.
2. Search for the selected target.
3. Approach the object.
4. Align using the wrist camera.
5. Grasp, return, and release.

Configured objects:

- `medicine_box`: ArUco ID `0`
- `glass`: ArUco ID `1`
- `tissue`: ArUco ID `2`

## Main Files

- `perception.py`: ArUco detection, depth sampling, and 3D projection.
- `state_machine.py`: Stretch Assist state machine.
- `accessible_ui.py`: Accessible target selector.
- `stretch_assist_config.json`: Runtime configuration.
- `tests/test_perception.py`: Perception tests.
- `tests/test_state_machine.py`: State machine tests.
- `tests/test_accessible_ui.py`: Accessible UI tests.
- `stretch_mujoco/models/scene.xml`: MuJoCo scene with the table, objects, and physical ArUco marker geometry.
- `stretch_toolkit/robocasa_config.json`: RoboCasa is disabled so the default scene is used.

## Relevant Commits

- `a20a703 feat: add stretch assist implementation`
  - Adds the initial perception module, UI, config, and state machine.
- `f0812da test: add stretch assist sim markers`
  - Adds simulated objects and markers to the scene.
- `95b7556 fix: remove stale stretch assist textures`
  - Removes references to missing PNG textures and models the ArUcos with geometry instead.
- `ca92471 fix: improve stretch assist simulator testing`
  - Adds `--no-teleop`.
  - Improves `Ctrl+C` handling.
- `6f84a1a fix: make stretch assist approach safer`
  - Prevents base rotation during search.
  - Makes approach more conservative to avoid hitting the table.
- `16288d5 fix: widen stretch assist head search`
  - Widens the head search sweep.
  - Adds `--debug-perception`.

## Validation Already Run

These commands have passed:

```bash
uv run pytest tests
uv run python -m py_compile perception.py state_machine.py accessible_ui.py tests/test_state_machine.py tests/test_perception.py tests/test_accessible_ui.py
```

Most recent result:

```text
9 passed
```

The MuJoCo XML was also validated after removing the missing texture references.

## Current Test Command

With perception debug output:

```bash
uv run python state_machine.py --target glass --no-teleop --debug-perception
```

Without debug output:

```bash
uv run python state_machine.py --target glass --no-teleop
```

## Observed Simulator State

The workflow can currently:

1. Start MuJoCo.
2. Load the default scene with the table, objects, and ArUcos.
3. Detect the glass using the head camera.
4. Transition from `SEARCH` to `APPROACH`.
5. Transition from `APPROACH` to `ALIGN`.

Observed example:

```text
[Stretch Assist] SEARCH: looking for glass
[Stretch Assist] APPROACH: found Glass
[Stretch Assist] ALIGN: close enough for wrist alignment
```

## Current Issues

### 1. The Simulator Prints Too Much FPS Warning Spam

This warning repeats throughout execution:

```text
WARNING: Passive viewer and camera rendering is below the requested 30.0FPS on the last render.
```

Impact:

- It clutters the terminal.
- It makes the useful Stretch Assist logs hard to read.
- It makes the program look stuck even when it is still progressing.

Likely cause:

- The MuJoCo passive viewer is requesting 30 FPS, but the machine cannot maintain that rate with the viewer plus RGB/depth camera rendering.
- There should be a way to lower `camera_rate`, disable this warning, or run in a quieter/headless mode.

Recommended next fix:

- Find where `camera_rate` is configured or where this warning is emitted in `stretch_mujoco`.
- Add an option such as `--quiet-sim`, or configure the viewer for a lower rate such as 10 FPS.
- Alternatively, filter only this specific warning without hiding real errors.

### 2. The System Takes Too Long or Loops in `ALIGN`

After detecting the glass and transitioning to `ALIGN`, the wrist camera does not see the ArUco:

```text
[Stretch Assist] perception debug: head_pan=-0.10 head_tilt=-0.77 visible=none
[Stretch Assist] SEARCH: wrist camera lost target
```

Then it returns to `SEARCH`, finds the glass again with the head camera, goes back to `ALIGN`, and repeats.

Impact:

- The robot never reaches `GRASP`.
- The system appears slow or stuck.

Main hypothesis:

- The ArUcos are placed on top of the objects, which makes them visible to the head camera but not necessarily to the wrist camera during alignment.
- The D405 wrist camera may be pointed in the wrong direction, too close, or too far away.
- The arm and wrist are not moved into a known pre-alignment pose before using the wrist camera.

Recommended next fix:

1. Before `ALIGN`, move the arm, lift, and wrist into a known pose where the D405 can see the tabletop.
2. Add visual debugging or save wrist camera frames.
3. Consider adding an extra ArUco that is visible from the side or at an angle, not only from above.
4. If the demo only needs to show search and approach, temporarily skip `ALIGN/GRASP` or simulate a successful grasp after head-camera approach.

### 3. `Ctrl+C` Can Still End with `ConnectionError`

Even though interrupt handling was improved, another traceback was observed when pressing `Ctrl+C` while MuJoCo was already shutting down:

```text
ConnectionError: The Stretch Mujoco Simulator is not running. Use the start() method to start it.
```

Impact:

- It does not affect the main logic, but it creates a poor shutdown experience during testing.

Likely cause:

- `KeyboardInterrupt` can happen while `step()` is inside a camera/perception call, and the next command send can happen after the simulator has already stopped.
- `_send_command(..., ignore_connection_error=True)` is used in `finally` and `abort`, but not for every shutdown path.

Recommended next fix:

- In `run()`, also catch `ConnectionError` if the simulator stops during the loop.
- In `step()`, if `_send_command(command)` fails with `ConnectionError`, transition to `ABORTED` without throwing a traceback.
- Keep real hardware errors visible outside shutdown paths.

### 4. Perception Debug Output Does Not Name the Camera

During `ALIGN`, debug output currently looks like this:

```text
[Stretch Assist] perception debug: head_pan=... head_tilt=... visible=none
```

However, during `ALIGN`, detection is coming from the wrist camera.

Impact:

- This is confusing during debugging.

Recommended next fix:

- Pass a camera name or state name into `_debug_visible_markers`.
- Print `camera=head` or `camera=wrist`.

## Current Architecture

Current states:

- `IDLE`
- `SEARCH`
- `APPROACH`
- `ALIGN`
- `GRASP`
- `RETURN`
- `RELEASE`
- `COMPLETE`
- `ABORTED`

Notes:

- The implementation tries to use `stretch_toolkit` instead of direct simulator internals.
- `--no-teleop` is required for autonomous simulator testing, because teleop can switch to `MANUAL` and override autonomous commands.
- `stretch_assist_config.json` is hot-reloaded at runtime.

## Recommended Next Session Priorities

Suggested order:

1. Silence or reduce the simulator FPS warning spam.
2. Make shutdown robust when using `Ctrl+C`.
3. Diagnose the wrist camera by saving frames or printing `camera=wrist`.
4. Add a pre-alignment pose before `ALIGN`.
5. Decide whether the demo should:
   - complete a realistic grasp using the wrist camera, or
   - simulate grasp after approach to provide a stable end-to-end demo.

## Useful Commands

Update the repo:

```bash
git pull origin main
```

Run tests:

```bash
uv run pytest tests
```

Compile the main files:

```bash
uv run python -m py_compile perception.py state_machine.py accessible_ui.py tests/test_state_machine.py tests/test_perception.py tests/test_accessible_ui.py
```

Run Stretch Assist:

```bash
uv run python state_machine.py --target glass --no-teleop --debug-perception
```
