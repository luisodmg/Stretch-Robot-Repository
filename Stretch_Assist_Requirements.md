# Stretch Assist: Technical Requirements

**Autonomous object retrieval and delivery through an accessible command interface**

| | |
|---|---|
| Course | Robotics and Cyber-Physical Systems |
| Institution | School of Sciences and Engineering, Tecnológico de Monterrey |
| Platform | Stretch MuJoCo Digital Twin (`stretch_mujoco_digital_twin`) |
| Team | Gustavo (technical lead, 80 h) and Luis (interface and evidence, 30 h) |
| Total effort | 110 h |
| Version | 1.1 |

## 1. Purpose

This document defines the functional requirements (RF) and non-functional requirements (RNF) of the project, breaks them down into tasks with an owner and an hour estimate, and keeps the traceability between tasks, requirements and deliverables. It is the technical basis to plan, build and justify each member's work.

## 2. Context and social justification

The Hello Robot Stretch is a mobile manipulator built for assistive robotics. Its most representative documented use is helping people with reduced mobility (quadriplegia, ALS, older adults) recover objects out of their reach, open doors or manipulate everyday objects at home. This project reproduces that core use case: the robot locates, picks up and delivers an object requested by the user through an accessible interface.

Development and testing happen entirely in the digital twin (MuJoCo), with no extra hardware and no cost. Moving to a physical robot is direct because the `stretch_toolkit` package exposes a unified control interface that selects the backend (simulation or real robot) automatically, with no code changes.

## 3. Scope

Target objects (three, fixed vocabulary):

| ArUco ID | Object | Description |
|---|---|---|
| 0 | Medicine box | Small cardboard box representing a medication container. |
| 1 | Glass | Cylindrical cup for drinking. |
| 2 | Tissue | Flat, lightweight cloth or paper tissue. |

Each object has an ArUco marker (ID 0, 1, or 2) attached as a sticker. The robot identifies the object from the marker ID and estimates its 3D position.

In scope:

* Visual detection of the three target objects using ArUco markers (ID 0–2) on the head camera RGB frame.
* 3D position estimation of the detected object by combining the marker centroid, the pixel depth from the depth camera, and the camera intrinsics.
* Autonomous search, approach, alignment, grasp, return and delivery.
* An accessible command interface (voice or large buttons) to select which of the three objects to retrieve.
* Manual operator override at any moment.
* Validation of the simulation-to-physical portability at the design and code level.

Out of scope:

* Mapping and path planning around complex obstacles. The robot uses a reactive approach rather than SLAM.
* Object recognition with deep learning or color (HSV). ArUco markers are the primary and only required detection method.
* Deployment on a physical robot during this project phase. The physical robot is a future extension: because the code uses only the `stretch_toolkit` API, running on the real Stretch requires no code changes. This is validated by design but not tested on hardware.

## 4. Actors

| Actor | Description |
|---|---|
| Assisted user | Person with reduced mobility who requests an object. Interacts only through the accessible interface. |
| Operator or caregiver | Person who can supervise and take manual control if something fails. |
| Robot (Stretch) | Runs perception, navigation and manipulation autonomously. |

## 5. Functional requirements (RF)

Each RF lists its priority, the owner, the task that implements it and a short justification.

| ID | Requirement | Priority | Owner | Task | Justification |
|---|---|---|---|---|---|
| RF-01 | The system must let the user select the target object through an accessible command (voice or large button). | High | Luis | T7 | It is the entry point that delivers the accessibility value. A user with reduced mobility cannot use a keyboard or gamepad. |
| RF-02 | The system must detect the target object by identifying its ArUco marker (ID 0 = medicine box, ID 1 = glass, ID 2 = tissue) in the RGB frame of the head camera using `cv2.aruco`. | High | Gustavo | T1 | ArUco markers are robust under the simulator's lighting conditions and transfer directly to the physical robot without recalibration. |
| RF-03 | The system must estimate the 3D position of the detected object by combining the ArUco marker centroid, the pixel depth from the depth camera, and the camera intrinsics. | High | Gustavo | T1 | Without a 3D position the robot cannot approach or grasp. The toolkit already exposes the depth frame and camera intrinsics. |
| RF-04 | The robot must run an autonomous search (rotating base and head) until it locates the object when it is outside the initial field of view. | High | Gustavo | T2 | The object will not always be in front of the robot, so the search makes the system genuinely autonomous. |
| RF-05 | The robot must approach the object autonomously using base velocity control. | High | Gustavo | T2 | The object is usually outside the arm reach, so the mobile base has to reposition. |
| RF-06 | The robot must perform fine alignment of the end effector through visual servoing with the wrist camera. | High | Gustavo | T3 | Base approach is not precise enough to grasp, so the wrist camera corrects the residual error. |
| RF-07 | The robot must close the gripper on the object and verify that the grasp succeeded. | High | Gustavo | T3 | Grasping without verification produces false successes, so verification enables retries (see RNF-05). |
| RF-08 | The robot must return to the user position and release or hand over the object. | High | Gustavo | T4 | This closes the assistance cycle. The goal is to deliver the object, not only to pick it up. |
| RF-09 | The operator must be able to take manual control (keyboard or gamepad) at any time, with priority over autonomous control. | High | Gustavo | T4 | Safety requirement for assistive applications. The toolkit already offers `merge_proportional`, where the human wins. |
| RF-10 | The system must give feedback about the robot state to the user (visual or audio: searching, grasping, delivering). | Medium | Luis | T8 | An assisted user needs to know what is happening, which improves trust and usability. |
| RF-11 | The full behavior must be orchestrated through a state machine (SEARCH, APPROACH, ALIGN, GRASP, RETURN, RELEASE). | High | Gustavo | T4 | It structures the system, eases debugging and allows error transitions. The toolkit includes `StateController`. |
| RF-12 | The system must run without code changes both in simulation and on the physical robot. | Medium | Gustavo | T5 | This is the explicit goal of being easy to apply to the physical robot, achieved by using only the `stretch_toolkit` API. |

## 6. Non-functional requirements (RNF)

| ID | Requirement | Category | Owner | Verification criterion |
|---|---|---|---|---|
| RNF-01 | Simulation to physical portability: the same script runs on both backends. | Portability | Gustavo | Code review confirms that only the `stretch_toolkit` API is used, with no MuJoCo specific calls in the control logic. |
| RNF-02 | Safety: operator control always has priority and a safe stop exists (Ctrl+C or clearing commands). | Safety | Gustavo | During an autonomous operation, a manual input interrupts and takes over the robot. |
| RNF-03 | Performance: the control loop runs at about 30 Hz with minimal perceptible latency. | Performance | Gustavo | Measure the real loop frequency and disable unused cameras if performance drops. |
| RNF-04 | Usability and accessibility: the interface is operable by a person with reduced mobility, without fine keyboard use. | Usability | Luis | Scenario test: complete a request using only voice or a single large button. |
| RNF-05 | Robustness: the system handles object not found and grasp failure with retries or a controlled abort. | Reliability | Gustavo | Hide the object and force a failed grasp, then confirm the system does not hang. |
| RNF-06 | Zero cost and no extra hardware: everything runs in the simulator with the laptop microphone. | Constraint | Luis | The full project runs without any purchase. |
| RNF-07 | Maintainability: modular code with parameters adjustable through a configuration file (JSON hot reload). | Maintainability | Gustavo | Changing a gain or a speed value in JSON takes effect without restarting. |
| RNF-08 | Reproducibility: the project installs and runs with `uv run` following the README on Windows, macOS and Linux. | Reproducibility | Gustavo | A third party clones the repo and runs the demo by following the document. |
| RNF-09 | Success metric: successful grasp and delivery rate of at least 70 percent over N attempts. | Quality | Luis | Log of N runs counting successes and failures. |

## 7. Task breakdown

The hours add up to 80 (Gustavo) plus 30 (Luis), for a total of 110. Each task references the requirements it satisfies.

### 7.1 Gustavo: technical core (80 h)

| Task | Description | Hours | Requirements | Partial deliverable |
|---|---|---:|---|---|
| T0 | Environment setup, repo study (`teleop_demo.py`, `visual_servoing.py`, `stretch_toolkit`) and a short literature review on assistive robotics. | 8 | RNF-08 | Working environment and architecture notes. |
| T1 | Perception module: ArUco marker detection for the three target objects (IDs 0–2) and 3D position estimation (marker centroid, depth, and camera intrinsics). | 14 | RF-02, RF-03 | `perception.py` with `detect_object(frame, depth_frame)` returning object name, ArUco ID, and 3D position. |
| T2 | Autonomous search and approach: rotate base and head to locate the object and use base velocity control to get closer. | 16 | RF-04, RF-05, RNF-03 | `search()` and `approach()` behaviors. |
| T3 | Grasp pipeline: visual servoing alignment with the wrist camera, gripper close and grasp verification. | 16 | RF-06, RF-07 | `grasp()` behavior with verification. |
| T4 | State machine integration (SEARCH to RELEASE) plus manual override and error transitions. | 12 | RF-08, RF-09, RF-11, RNF-02, RNF-05 | `state_machine.py` running end to end. |
| T5 | Simulation to physical validation, gain tuning through JSON and a test matrix. | 8 | RF-12, RNF-01, RNF-07 | Compatibility report and tuned config. |
| T6 | Technical documentation and the technical section of the final report. | 6 | Deliverables | Technical section of the report. |
| | **Subtotal Gustavo** | **80** | | |

### 7.2 Luis: interface and evidence (30 h)

| Task | Description | Hours | Requirements | Partial deliverable |
|---|---|---:|---|---|
| T7 | Accessible command interface: offline voice recognition (Vosk or `speech_recognition`) or a large button GUI that emits the target object. | 12 | RF-01, RNF-04 | `accessible_ui.py`. |
| T8 | State feedback to the user plus accessibility design and use scenarios with personas. | 6 | RF-10, RNF-04 | Feedback module and scenarios document. |
| T9 | Testing and data collection: run N trials, log success rate, failure cases and override evidence. | 6 | RNF-06, RNF-09 | Results table (successes and failures). |
| T10 | Evidence production: demonstration video, screenshots (RoboCasa and result) and report sections. | 6 | Deliverables | Video, screenshots and report sections. |
| | **Subtotal Luis** | **30** | | |

## 8. Responsibility matrix

R means responsible, C means collaborates, I means informed.

| Activity | Gustavo | Luis |
|---|:---:|:---:|
| Architecture and technical design | R | C |
| Perception and 3D estimation | R | I |
| Autonomous navigation and grasping | R | I |
| State machine and integration | R | C |
| Accessible interface and user feedback | C | R |
| Testing, metrics and data | C | R |
| Video, screenshots and report | C | R |
| Simulation to physical validation | R | I |

## 9. Acceptance criteria

The project is considered complete when:

1. The user requests an object through the accessible interface and the robot delivers it autonomously (RF-01 to RF-08, RF-11).
2. The operator can interrupt and take manual control at any time (RF-09, RNF-02).
3. The system correctly handles object not found and grasp failure without hanging (RNF-05).
4. A code review shows that the control logic does not depend on the simulation backend (RF-12, RNF-01).
5. The recorded success rate is at least 70 percent over the defined N runs (RNF-09).

## 10. Deliverables

* Repository with the modules `perception.py`, `state_machine.py`, `accessible_ui.py` and the JSON configuration.
* Demonstration video of the full retrieval and delivery cycle.
* Screenshots of the RoboCasa environment and the task result.
* Test results table (success rate and failure cases).
* Final report with the social justification, the architecture, the description of the nine degrees of freedom used and the simulation to physical compatibility analysis.

## 11. Risks and mitigation

| Risk | Impact | Mitigation |
|---|---|---|
| ArUco marker not visible (occlusion or angle). | Medium | The SEARCH state rotates the base and head until the marker enters the camera field of view. Maximum search angle is configurable via JSON. |
| Imprecise grasp from depth error. | Medium | Wrist visual servoing as fine correction (RF-06) plus verification and retry (RF-07, RNF-05). |
| ArUco texture not rendering clearly in MuJoCo. | Low | Generate the marker texture at high resolution (at least 200×200 px) and verify detection in simulation before proceeding to physical testing. |
| Slow RoboCasa asset download (about 5 GB). | Low | Download at the start (T0). The core of the project also works in the simple block environment. |
| Low performance with several cameras active. | Low | Enable only the cameras needed per state (RNF-03). |
| Unreliable voice recognition. | Medium | Use a closed vocabulary (three object names) and provide a button GUI as an equivalent alternative (RF-01). |

## 12. Glossary

* **Digital twin:** simulated replica of the physical robot used for development and testing.
* **Visual servoing:** controlling motion using real time camera feedback.
* **ArUco marker:** a fiducial square marker with a unique binary ID that OpenCV can detect and identify in a camera image. Used here as the primary object identification method.
* **Sim-to-real:** transfer of a behavior developed in simulation to the physical robot.
* **State machine:** model that organizes behavior into defined states and transitions.
