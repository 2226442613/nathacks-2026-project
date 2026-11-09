Hospital Guardian
A hospital-environment cleaning robot hackathon MVP combining a fixed camera, visual localization, computer-side navigation, and an ESP32 robot controller.

The computer detects new spills, converts image pixels into scene coordinates, tracks the robot, and plans an approach. It sends duration-limited movement commands over Wi-Fi while the camera provides actual position and heading feedback. Once the robot arrives, the ESP32 executes a complete cleaning sequence. The computer waits for the completion event, then guides the robot back to HOME. A Dashboard displays annotated video, task queues, history, and progress.

Current scope: Modules 01–07, Dashboard, offline simulation, and supervised physical operation. Red/non-red classification selects demonstration priority and fluid type; it does not identify medical substances. Clinical cleaning effectiveness has not been validated.

Documentation reflects the supplied project materials dated November 9, 2026. Simulation results, individual hardware checks, and complete physical end-to-end validation are recorded separately.

Project Access and Documentation Guide
This README combines the Hospital Guardian computer application and the NatHack Robot ESP32 firmware. The computer runs vision, navigation, the Dashboard, and a TCP client. The ESP32 runs the TCP server and controls motors, pumps, and servos. The Dashboard's Flask web server is a separate display service.

Computer application: Run commands from the project root containing main.py, vision/, navigation/, and robot_comm/. The supplied computer-side README did not specify a local source directory or public repository URL.
Robot firmware: On the development computer, the independent source folder is /Users/meilinz/Documents/Arduino/NatHack_Robot/. Open NatHack_Robot.ino in Arduino IDE. This local path is not a public download link.
Dashboard: After starting the web service, open http://127.0.0.1:5050 on the same computer. This is a local address, not a public deployment.
Robot connection: Use the ESP32's current LAN IP and TCP port 3333. The example 172.20.10.12 is a previously used address; check Serial Monitor for the current address.
Submission materials: Supply both source projects alongside this README. Include Secrets.example.h and exclude the private Secrets.h. No public repository, slide deck, or demo-video link was supplied.
Sections 1–11 cover the computer application, Section 12 explains integration, and Section 13 provides robot hardware and firmware details. Relative documentation links resolve from the computer project root and require the accompanying files.

Computer-side features, accuracy figures, and test counts below come from the supplied README; they were not rerun for this document. Robot details were checked against the independent firmware folder's README, main sketch, configuration, and protocol source. No hardware was operated or reflashed during documentation preparation.

1. System Architecture
TEXT
复制
Fixed camera (iPhone Continuity Camera / USB camera)
  → Module 02: detect, classify, and confirm new spills across frames
  → Module 03: spill pixels → floor/wall coordinates
  + Module 04: ArUco → robot position and heading
  → Module 05: parking pose, tool geometry, and reachability
  → Module 06: closed-loop navigation, arrival checks, cleaning wait, return
  → Module 07: TCP 3333, ASCII commands, asynchronous replies
  → ESP32: MOVE execution and complete cleaning presets

Module 01 provides scene calibration.
Dashboard displays video, tasks, and progress.
Module	Responsibility	Main source
01 · Camera Calibration	Camera capture, independent floor/wall calibration, coordinate conversion	vision/camera.py, vision/calibration.py, main.py
02 · Spill Detection	Compare against the registered valid background; detect new regions; RED/NON_RED classification; candidate/confirmed states	vision/spill_detection.py, vision/spill_tracking.py
03 · Surface Localization	FLOOR/WALL/BOUNDARY classification; coordinates, contours, dimensions, and quality	vision/localization.py, vision/target_geometry.py
04 · Robot Localization	Single ArUco marker, robot center/heading, measured fixed-height mapping, pose freshness	vision/robot_tracking.py, vision/robot_plane.py
05 · Cleaning Geometry	Separate floor/wall tools, approach poses, contact coverage, reachability	vision/cleaning_planner.py, navigation/adapters.py
06 · Navigation	Y→X outbound route, camera feedback, MOVE/STOP, corrections, cleaning handoff, X→Y return	navigation/controller.py, navigation/automatic.py
07 · Communication	CAPS negotiation, ASCII framing, actual execution outcomes, logs, simulator	robot_comm/
Dashboard	DEMO/LIVE annotated video, task queue, history, progress	dashboard/
The computer does not individually sequence pumps, sponge movements, servos, or cleaning strokes. Target coordinates remain on the computer. The ESP32 receives surface/fluid labels, performs the preset, and reports completion.

2. Implementation and Validation Status
Automatic detection, red-first priority, localization, parking planning, closed-loop navigation, cleaning handoff, and return have been implemented in simulation.
Module 04 has an operator-confirmed physical localization PASS using four training points and three independent validation points: RMS 1.045 cm, maximum error 1.253 cm. Saved mapping errors agree with these figures. Validity is limited to the corresponding fixed camera, marker height, and scene.
Physical TCP, PING/CAPS, and MOVE execution records exist. These do not establish a successful physical detection→cleaning→return run. The latest automatic-run record was NO_REACHABLE_TARGET; see the skipped-target investigation.
Dashboard LIVE reuses the OpenCV annotated frames from test-vision and streams them as MJPEG. It does not implement a separate JavaScript vision annotation pipeline.
Physical tool measurements remain incomplete. The floor MVP can invoke the ESP32 preset, but its checks cover parking and body/tool collision envelopes, not verified contact or spray coverage. Wall plans remain rejected when precise contact geometry is missing.
3. Environment and Installation
The primary development environment is macOS on Apple Silicon, with the camera exposed as a local macOS device. Python 3.12 is recommended; the original project also supports Python 3.11. A Raspberry Pi, separate iPhone video server, CUDA, and deep-learning framework are not required.

Run from the computer project root:

BASH
复制
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements-dashboard.txt
python main.py --help
requirements-dashboard.txt installs the base dependencies plus Flask. For vision/navigation CLI use only, install requirements.txt instead. Dependencies include OpenCV contrib, NumPy, and pytest; Dashboard additionally uses Flask.

Install only opencv-contrib-python, without another OpenCV package providing cv2. Native windows such as test-vision require the non-headless version. On first use, grant camera permission to the application launching Python.

All following computer-side commands assume the repository root and an activated .venv. Otherwise, replace python with .venv/bin/python.

4. Quick Start: Display and Simulation
4.1 Dashboard DEMO — No Camera or ESP32
Use the bundled synthetic scene calibration on a fresh checkout:

BASH
复制
python main.py dashboard --mode demo \
  --calibration data/module03_demo/calibration.json --port 5050
Open the local Dashboard and use Play/Step to inspect simulated detection, the priority queue, and task history. DEMO opens neither a camera nor a robot connection. Dashboard startup currently still reads a calibration file, so use the synthetic file above on a new checkout. An existing local installation can use its default calibration.

4.2 OpenCV Combined Vision — Camera Only
Prerequisite: matching scene calibration, background reference, and robot mapping files must already exist locally.

BASH
复制
python main.py test-vision --camera 0
One capture pipeline feeds spill detection and ArUco tracking. The display includes:

Robot marker outline, ID, center, X/Y, heading, and localization state.
Spill boxes, IDs, RED/NON_RED labels, candidate/confirmed states, and coordinates.
FPS, geometry status, and specific error reasons.
Coordinates print approximately once per second. Press Q to exit. This command does not connect to the ESP32.

4.3 Dashboard LIVE — Annotated Camera Video
Close test-vision, camera previews, and other applications using the same camera first:

