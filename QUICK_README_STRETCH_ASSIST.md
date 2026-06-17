# Quick README - Stretch Assist

## What We Are Working On

We are building a Stretch Assist prototype for the Hello Robot Stretch platform. The goal is for a user to select a simple object, such as a medicine box, a glass, or a tissue box, and have the robot detect it, approach it, align its arm, grasp it, return, and release it.

The current version is being tested in the MuJoCo simulator first. The code is structured around `stretch_toolkit`, so the same high-level logic can later be moved toward the physical Stretch robot with fewer simulator-specific changes.

## What We Programmed

- `perception.py`: computer vision module. It detects ArUco markers in RGB images, filters the target IDs, samples depth, and estimates the object's 3D camera-frame position.
- `state_machine.py`: autonomous behavior state machine. It coordinates `SEARCH`, `APPROACH`, `ALIGN`, `GRASP`, `RETURN`, and `RELEASE`.
- `vision_window.py`: live robot-vision window. It shows the camera feed, detected marker boxes, marker IDs, object labels, and estimated distance.
- `accessible_ui.py`: simple accessible target selector with large controls.
- `stretch_assist_config.json`: runtime-editable configuration for speeds, tolerances, timeouts, pose targets, and grasp parameters.
- `tests/`: automated tests for perception, state-machine behavior, and UI logic.

## What We Changed In The Simulator

- Added simulated target objects with ArUco markers:
  - ID `0`: `medicine_box`
  - ID `1`: `glass`
  - ID `2`: `tissue`
- Adjusted the scene so the objects are placed on the robot's right side, where Stretch's arm can reach them more naturally.
- Updated the approach behavior so the robot drives forward and stops by odometry at a reachable x-position.
- Added a scripted side-grasp sequence: pregrasp, arm extension, gripper close, lift, return, and release.
- Fixed control signs for simulated joints such as head and wrist joints.
- Reduced simulator warning noise so the demo output is easier to explain.

## Why There Are ArUcos On The Robot And Scene

There are two different types of ArUco markers visible in the simulator:

- Existing robot/model markers: the Stretch model already includes ArUco stickers on parts such as the base, shoulder, wrist, gripper camera, and gripper fingers. These are part of the simulated robot model and are useful as visual reference markers for calibration, localization, debugging, or robot-part pose references.
- Stretch Assist object markers: these are the markers we added on the table objects. These are the ones used by our task logic: ID `0` for the medicine box, ID `1` for the glass, and ID `2` for the tissue box.

The marker on the docking station is also part of the existing simulated environment. It can be used as a docking/localization reference, but it is not one of our object-retrieval targets.

Technically, this does not confuse the task because `perception.py` filters detections by the selected target ID. The state machine only reacts to the expected object marker, not every ArUco that appears in the camera view.

## What We Have Achieved

- The robot can visually detect marked objects using ArUco markers.
- The system can distinguish between the medicine box, glass, and tissue box by marker ID.
- A complete autonomous state-machine flow exists from search to release.
- The live vision window makes it clear what the robot is detecting during the demo.
- The main modules have automated tests.
- Behavior parameters can be tuned through JSON without rewriting the main control code.

## Technical Explanation Of The Vision System

The vision system uses OpenCV ArUco marker detection. This is useful for a prototype because each object has a known visual ID, making the perception problem more reliable and easier to debug than general object recognition.

The pipeline is:

1. The robot camera provides an RGB frame.
2. OpenCV detects ArUco markers in the frame.
3. The system filters only the IDs used by Stretch Assist.
4. The marker center is computed in pixel coordinates.
5. The depth image is sampled near that center.
6. Camera intrinsics are used to project the pixel and depth into a 3D camera-frame point.
7. The state machine uses that detection to decide whether to keep searching, approach, align, or grasp.

## Technical Explanation Of The Behavior

The robot behavior is organized as a state machine instead of one long script. This makes the system easier to test and explain.

- `SEARCH`: moves the head through a search pattern until the target marker is visible.
- `APPROACH`: drives the base toward a reachable position while keeping the target confirmed.
- `ALIGN`: prepares the robot for manipulation.
- `GRASP`: executes a scripted manipulation sequence using lift, arm extension, wrist, and gripper commands.
- `RETURN`: moves the robot back toward its starting pose.
- `RELEASE`: opens the gripper to drop or hand over the object.
- `COMPLETE`: finishes the request.

Manual teleoperation can override autonomous commands, which is important for safety and debugging.

## Short Presentation Script

"Our project is a Stretch Assist prototype for retrieving simple objects. We implemented a perception module using OpenCV and ArUco markers, a state machine that controls the autonomous workflow, and a live vision window that shows what the robot sees. In simulation, the robot can search for a selected object, identify it by marker ID, approach it, and execute a gripper-based grasp sequence. The grasp is still being tuned for physical consistency, but the main perception, control, configuration, and autonomous behavior architecture is already implemented and tested."

## Possible Professor Questions About Functionality

**What exactly did you program?**  
We programmed the ArUco-based perception pipeline, the autonomous state machine, the live robot-vision debug window, the accessible object-selection UI, and the JSON-based tuning configuration.

**How does the robot know which object to pick?**  
Each object has an ArUco marker with a unique ID. The user selects a target, and the perception module searches for that specific ID.

**Why does the robot have ArUco markers on its own body?**  
Those markers come from the Stretch simulation model. They are not the objects we are trying to retrieve. They are visual fiducials attached to robot links, often useful for calibration, pose reference, debugging, or validating camera views.

**What is the final ArUco marker away from the objects?**  
That marker belongs to the docking station/environment model. It is a reference marker for the simulated scene, not one of our retrieval targets.

**How do you avoid detecting the wrong ArUco?**  
The perception module filters by allowed target IDs. For Stretch Assist, only IDs `0`, `1`, and `2` represent target objects, and when the user selects one object the system searches for that specific ID.

**Why use ArUco markers instead of object detection with AI?**  
ArUco markers are deterministic, easy to test, and provide reliable IDs for a robotics prototype. This lets us focus on integrating perception, control, and manipulation before moving to more complex object recognition.

**How is distance estimated?**  
The system detects the marker center in the RGB image, samples the aligned depth frame near that pixel, and uses camera intrinsics to estimate a 3D point relative to the camera.

**What happens if the target is not visible?**  
The robot stays in `SEARCH`, moving the head through a sweep pattern until the marker is detected or a timeout/failure condition occurs.

**How does the state machine help?**  
It separates the behavior into clear phases. Each state has a specific job and transition condition, which makes the system easier to debug, test, and explain.

**How does the robot approach the object?**  
In the current simulator setup, the robot approaches by driving forward and stopping at a configured base x-position for the selected target. This keeps the side-grasp geometry predictable.

**How does the grasp work?**  
The grasp is currently scripted. The robot moves into a pregrasp pose, extends the arm, lowers or positions the gripper near the object, closes the gripper, lifts, returns, and releases.

**Is the grasp fully reliable?**  
Not yet. The full state-machine flow works in simulation, but the exact physical grasp still needs tuning so the object is lifted and carried consistently every time.

**What role does the wrist camera play?**  
The wrist camera is intended for close-range alignment near the object. The current most explainable and stable demo focuses on head-camera detection plus scripted manipulation, while wrist-camera alignment is still an area for improvement.

**How can you tune the robot without changing code?**  
Parameters such as speeds, timeouts, target positions, and grasp poses are stored in `stretch_assist_config.json`. They can be edited to tune behavior without modifying the main Python logic.

**Why test in simulation first?**  
Simulation lets us validate perception, state transitions, and control logic without risking hardware damage. It also makes failures easier to reproduce.

**Can this run on the real Stretch robot?**  
The architecture is designed for that direction because it uses `stretch_toolkit` abstractions instead of direct MuJoCo calls. However, real-hardware deployment still requires camera calibration checks, safety validation, and manipulation tuning.

**What safety or override mechanism exists?**  
Manual teleoperation can be merged over autonomous commands, so an operator can override the robot during testing.

**What are the main limitations right now?**  
The current system depends on visible ArUco markers, the grasp is still being tuned, and wrist-camera alignment needs more robustness.

## How To Run The Demo

Render on the NVIDIA GPU with the launcher (recommended on this laptop):

```bash
./run_stretch_assist.sh --target glass --destination table --no-teleop
```

The robot searches for the object, approaches, grasps it, carries it to the
chosen destination, and places it on top. The target and destination can be
changed:

```bash
./run_stretch_assist.sh --target medicine_box --destination shelf --no-teleop
./run_stretch_assist.sh --target tissue --destination person --no-teleop
```

Targets: `medicine_box`, `glass`, `tissue`. Destinations: `table`, `shelf`,
`person`.

### Interactive Mode

Give repeated "grab X, drop at Y" commands without restarting. The robot
remembers where it left each object and goes straight there next time, instead
of returning to the start:

```bash
./run_stretch_assist.sh --interactive --no-teleop
```

A large-button selector asks for the object, then the destination, after each
delivery. Close the selector or press Ctrl-C to quit.

With the vision window enabled, the demo can visually show what the robot detects and how the state machine progresses.

## Current Status

The system is a functional simulation prototype. The strongest completed part is the integration between vision and autonomous state control. The next step is improving grasp consistency and preparing the system for future testing on the physical Stretch robot.
