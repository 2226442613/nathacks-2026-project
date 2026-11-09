# Module 07 delivery: inspected ESP32 text communication

**Historical PWM-protocol report.** Current PC/simulator behavior and the shared
duration/safety contract are in [MODULE07_DURATION_PROTOCOL.md](MODULE07_DURATION_PROTOCOL.md).
The old staged firmware patch below is unchanged and incompatible with the new
motion gate; it is not the firmware update for this delivery.

Implemented host client, local simulator, exact wire logging, CLI integration,
regression tests and a focused **staged firmware patch**. The original firmware
in `/Users/williamwu/Downloads/NatHack_Robot` was inspected and remains unchanged.
No firmware was uploaded, no physical ESP32 endpoint was contacted, and no physical
motor, pump or servo was operated. Host/simulator code is implemented; hardware
compatibility and target-board build remain unverified pending the steps below.

## Inspected firmware and compatibility assessment

| Area | Actual source behavior |
| --- | --- |
| Entry/framework | `NatHack_Robot.ino`, Arduino sketch with `.cpp/.h` files; no PlatformIO/ESP-IDF build manifest or pinned board/core versions |
| Board | README describes ESP32-S3; exact FQBN/board variant is not specified |
| Wi-Fi | STA, existing Config.h SSID/password, sleep disabled, reconnect attempt every 10 seconds; not an AP |
| Network | Single TCP client, configurable `NET_PORT`, currently 3333, newline strings; no HTTP/WebSocket/Bluetooth endpoint |
| Console | Serial 115200; same command parser; serial debug goes to Serial, not arbitrarily onto the Wi-Fi stream |
| Parser | `Protocol.cpp`; trims/uppercases; comma-separated plain ASCII commands |
| Drive | Four independent motor channels, differential/skid steering; F/B translation, L/R counter-rotation; raw PWM 0..255, no cm/s conversion or encoder position feedback |
| Manual motion timeout | `MotorControl.cpp`: 450 ms after last timed MOVE; cleaner's internal strokes use untimed motor commands |
| Existing cleaning | `TARGET` stores surface/fluid; `ARRIVED` starts the pre-existing open-loop timed cleaning sequence; `DONE,CLEANING` reports software sequence completion |
| Stop/disconnect | STOP/ABORT calls cleanerAbort: motors zero, both pumps off, servos home; detected TCP disconnect already aborts |
| Missing original safety | No heartbeat command, no all-actuator silent-connection watchdog, no latched ESTOP/reset, no command IDs; oversized reader reset could interpret a trailing suffix as another command |
| Build tools | No arduino-cli, pio, Arduino IDE, Arduino15 ESP32 core cache or PlatformIO installation found in inspected standard locations |
| Dependencies | Arduino ESP32 board core, WiFi, ESP32Servo; actual board/core/library versions still need confirmation |

The source itself labels the pin plan **EXAMPLE ONLY**, not verified wiring.
The inspected mapping is:

| Function | Source mapping |
| --- | --- |
| Front-left motor IN1/IN2/PWM | GPIO 1/2/11, invert=false |
| Rear-left motor | 3/4/12, invert=false |
| Front-right motor | 5/6/13, invert=false |
| Rear-right motor | 7/8/14, invert=false |
| Driver standby | 15 and 16; source drives HIGH |
| Fluid pump A/B | 17/18, active HIGH, external driver inputs; mutually exclusive |
| Shared spray-tube lift servo | 9, 50 Hz, attach range 500..2400 us, home 0°, wall preset 90° |
| Wall-sponge servo | 10, same servo frequency/range, home 0°, maximum 180° |

README describes two TB6612FNG drivers, two fluid-select pumps, a passive ground
sponge and a sweeping wall-sponge servo. It explicitly flags actual wiring,
pump count, timings and mechanical ranges as needing physical verification.
Config: MAX_PWM=255, CLEAN_DRIVE_PWM=110; spray 1200 ms, servo settle 700 ms;
ground strokes 350 ms each ×3 cycles; wall servo 2° every 15 ms ×3 sweeps.
No pin, PWM setup, direction inversion, actuator setup, cleaner timing or hardware
limits were changed by the staged patch. No Wi-Fi credentials are copied into
this repository or printed in logs.

## Plain-text protocol

Primary transport remains local Wi-Fi TCP, default port **3333**. Host/port are
configurable. Each network application message is ASCII text plus LF; incoming
firmware CRLF is supported. Limit: 120 ASCII content bytes per line. No network
JSON, cloud service, physical velocity conversion or competing transport.

Preserved commands:

| Wire line | Response/meaning |
| --- | --- |
| `PING` | `PONG` |
| `STATUS` | `STATUS,<cleaner state>` |
| `MOVE,F,80` / B / L / R | `OK,MOVE`; raw PWM 0..255; drive command accepted, not distance completed |
| `TARGET,GROUND,BLOOD` / GROUND,OTHER / WALL,BLOOD / WALL,OTHER | `OK,TARGET` or `ERR,BUSY` |
| `ARRIVED` | `EVENT,CLEANING_STARTED`, then `OK,ARRIVED`; later `DONE,CLEANING` |
| `STOP` or `ABORT` | `OK,STOP`; stop motors/pumps, home servos, abort target |

Original detailed errors are retained: ERR,MOVE_FORMAT, ERR,DIRECTION,
ERR,SPEED, ERR,SPEED_RANGE, ERR,TARGET_FORMAT, ERR,UNKNOWN_TARGET,
ERR,BUSY, ERR,NO_TARGET and ERR,UNKNOWN_COMMAND.

Additive safety patch commands:

| Wire line | Response/meaning |
| --- | --- |
| `CAPS` | `CAPS,NATHACK_V1_1,MOVE_PWM_0_255,STOP,ESTOP,RESET_ESTOP,HEARTBEAT,SAFETY,TARGET,ARRIVED` |
| `HEARTBEAT` | `OK,HEARTBEAT` |
| `SAFETY` | `SAFETY,ESTOP=0,WATCHDOG_MS=1000,WATCHDOG=OK` (ESTOP 0/1; watchdog OK/EXPIRED) |
| `ESTOP` | `OK,ESTOP`; abort and latch locally |
| `RESET_ESTOP` | `OK,RESET_ESTOP`; explicitly clear latch but leave stopped, never resume a previous task |

A latched controller rejects movement/target/ARRIVED with ERR,ESTOP_LATCHED.
STOP, diagnostics and heartbeats cannot clear the latch. TCP reconnect does not
clear it. The latch is RAM state; this patch does not claim persistence through
power loss/reboot or replacement of a physical power cutoff.

The patched firmware independently aborts all existing actuators after >1000 ms
without a valid communication refresh, emits EVENT,WATCHDOG_STOP, and keeps the
450 ms manual-drive timeout. Valid legacy commands also refresh communication,
so an existing controller can maintain liveness with PING/STATUS/MOVE. Serial-only
cleaning now also requires periodic valid communication. Invalid input does not
refresh the watchdog. The SAFETY watchdog field describes current liveness;
valid queries refresh liveness, and watchdog events remain in the wire log.

Oversized/invalid-ASCII lines produce ERR,LINE_TOO_LONG / ERR,ENCODING, abort, and
are discarded **through newline**, never execute their suffix. The host and
simulator fail closed by disconnecting on malformed framing. Simulator uses the
same command/reply vocabulary; it closes on the malformed line instead of keeping
the stream open as the firmware does. Both stop rather than executing its tail.
Partial packets and multiple lines per packet are handled.

`STATUS,IDLE` describes the cleaner state, **not measured motor standstill**:
manual MOVE can run while cleaner state is IDLE. No real actuator/encoder feedback
exists. Simulator INFO lines expose its simulated commanded PWM/pump/servo state;
these are not invented physical telemetry or camera poses.

## Command lifecycle and client API

The protocol has no IDs. We deliberately preserve working command names rather
than introduce an incompatible envelope. Thus there is no exactly-once delivery,
persistent duplicate cache or uniquely correlated physical completion guarantee.
The client serializes ordinary request/reply exchanges. It never automatically
retries motion, actuator commands or failed connections. Explicit reconnect starts
disarmed and never replays previous commands. Repeated MOVE is a deliberate manual
refresh, not interpreted as a second distance move.

- TX: socket sendall accepted the complete line; partial-send failures are errors,
  not a successful TX. This is transport acceptance only.
- OK: firmware accepted the command. `CommandResult.firmware_accepted` reflects
  this; ERR is a rejection, not a transport success claim about execution.
- EVENT,CLEANING_STARTED: legacy cleaner software started its sequence.
- DONE,CLEANING: firmware reports timed sequence completion, not camera-observed
  movement or verified successful cleaning. It is separate telemetry, never an ACK.
- `CommandResult.execution_completed` remains false; no inferred physical completion.

`robot_comm.RobotClient` exposes connect, disconnect, send_command, send_stop,
emergency_stop, reset_estop, request_status, receive_telemetry, is_connected and
get_last_error. The independent package imports no OpenCV or vision module;
`python -m robot_comm` is usable without the vision application's imports.

Reader and heartbeat threads keep reception/liveness separate from the caller.
Time-bounded requests fail/close on timeout or loss and reject new commands.
STOP/ESTOP bypass the ordinary request lock, cancel queued unsent movement and
interrupt outstanding waits.
Their immediate result is **transport accepted, firmware acceptance unknown**;
raw OK replies are still logged/queued. A PING response barrier drains prior replies
before the next ordinary exchange; an ambiguous error fails closed. This is a
best-effort network priority, not preemption of bytes already sent or a physical
emergency-stop guarantee.

Heartbeat defaults to 0.2 seconds, timeout 0.8 seconds. Failure closes the socket,
disarms and wakes pending operations. Reconnection is explicit; the host's ESTOP
latch also persists across reconnect until an explicit reset. Firmware independently
stops even if the MacBook process or logging fails.

## Simulation and physical gates

Simulation is default, restricts connections to loopback, and never reads robot
plans or localization to generate commands. FirmwareModel simulates the inspected
raw-PWM directions and existing timed ground/wall cleaner. It models pumps A/B,
shared spray lift and wall-sponge sweep; the server prints these as SIMULATED
commanded states. No displacement/pose is derived from PWM.

Physical mode requires an explicit mode and host and remains DISARMED. Raw movement
requires all of: patched capability/safety reports, no ESTOP, explicit `--arm`,
`--authorize-motion`, and an operator-supplied **physically validated** maximum PWM
via `--validated-max-pwm`. Supplying a number is an operator attestation, not a
software motor calibration. Requested PWM must stay within that bound. Reconnect,
STOP, ESTOP and reset disarm. The client cannot confirm that the firmware's pin
plan or mechanical hardware is correct; those must be checked physically first.

Original firmware can still be queried and stopped by this client, but cannot be
armed. emergency_stop on legacy firmware sends STOP with a warning and does not
falsely claim hardware latching. RESET_ESTOP is unsupported there.

Physical TARGET/ARRIVED are deliberately blocked even after arming: they trigger
the pre-existing full physical cleaning sequence, which is outside Module 07.
They are available in simulation. No autonomous plan execution was implemented.

Unsupported independent commands remain unsupported: floor/wall pump channels
(the firmware selects fluid, not independent surfaces), continuously rotating
sponge motor, arbitrary nozzle height in cm, independent nozzle angle command,
move-distance, physical cm/s, encoder feedback. Do not use conceptual commands
such as FLOOR_SPRAY ON or WALL_NOZZLE_HEIGHT 12; the host rejects them before TX.

## Exact CLI commands

Open two terminals in `/Users/williamwu/natHack`. Log files must be new; choose a
new filename for repeat runs. Every network TX/RX prints and flushes by default,
with millisecond timestamps; the optional file contains the same plain text.
No Wi-Fi provisioning/credential command exists.

Terminal 1 — local simulator:

```bash
.venv/bin/python main.py robot-simulator --host 127.0.0.1 --port 3333 \
  --log-file data/module07_simulator.log
```

Terminal 2 — safe PING/STATUS and telemetry:

```bash
.venv/bin/python main.py robot-client --mode simulation --host 127.0.0.1 \
  --port 3333 --command PING --command STATUS --listen 1 \
  --log-file data/module07_client.log
```

Simulation-only movement, wall lift/pump/sponge, STOP, ESTOP, reset:

```bash
.venv/bin/python main.py robot-client --mode simulation --port 3333 --demo --listen 1
```

Individual simulated raw commands:

```bash
.venv/bin/python main.py robot-client --port 3333 \
  --command MOVE,F,80 --command STATUS --command STOP --listen 0.5
```

The standalone no-OpenCV alternative uses exactly the same arguments:

```bash
.venv/bin/python -m robot_comm robot-simulator --port 3333
.venv/bin/python -m robot_comm robot-client --port 3333 --command PING --command STATUS
```

A physical **read-only diagnostic** template, after obtaining the real IP, is:

```bash
.venv/bin/python main.py robot-client --mode physical --host "$ESP32_HOST" \
  --port 3333 --command PING --command STATUS --listen 1
```

This is disarmed. No physical commands were run during development. Future manual
PWM bench testing additionally requires the explicit gates described above; it is
not enabled by an ArUco detection or an UNVERIFIED pose. A network stop cannot
replace a physical emergency stop. LAN protocol is unauthenticated/unencrypted;
no cloud/network-exposure or physical security claim is made.

Actual localhost demo artifacts are in `data/module07_demo/client.log` and
`simulator.log`. Reproduce in a fresh directory:

```bash
.venv/bin/python -m tools.generate_communication_demo --output data/module07_demo_new
```

Excerpt from the executed client transcript:

```text
[18:26:20.895] [TX] MOVE,F,80
[18:26:20.895] [RX] OK,MOVE
[18:26:23.700] [TX] STATUS
[18:26:23.700] [RX] STATUS,WALL_WIPE
[18:26:23.700] [TX] ESTOP
[18:26:23.701] [RX] OK,ESTOP
[18:26:23.701] [TX] SAFETY
[18:26:23.701] [RX] SAFETY,ESTOP=1,WATCHDOG_MS=1000,WATCHDOG=OK
```

These timestamps are software-only demo evidence, not physical robot execution.

## Firmware patch and build status

The source firmware is outside the managed writable workspace. Instead of changing
it or requesting an unnecessary broad filesystem approval, this delivery stages:

- Modified replacements: `firmware/module07/NatHack_Robot.ino`, `Protocol.cpp`.
- New files: `CommunicationSafety.cpp`, `CommunicationSafety.h`.
- Unchanged reference: `Protocol.h`.
- Reviewable unified patch: `firmware/module07.patch`.
- Original source fingerprints: `firmware/module07/original_sha256.json`.

No Config.h, MotorControl, Actuators or Cleaner changes are in the patch. Credentials,
pins, directions, PWM initialization, actuator initialization and original cleaning
sequences remain in the existing project. Old valid commands remain supported;
the independent all-actuator watchdog is an intentional additional safety constraint.

Before applying, check the source fingerprints. Preview against the original:

```bash
patch --dry-run -d /Users/williamwu/Downloads/NatHack_Robot -p1 \
  -i /Users/williamwu/natHack/firmware/module07.patch
```

After reviewing the changes, the corresponding manual application command is:

```bash
patch -d /Users/williamwu/Downloads/NatHack_Robot -p1 \
  -i /Users/williamwu/natHack/firmware/module07.patch
```

These commands were **not run against the original directory**. Dry-run and actual
application were verified in a temporary copy; patched files matched the staged
sources exactly. Original file hashes were checked unchanged.

Target build is **not performed**: arduino-cli/PlatformIO/Arduino IDE and the board
core are unavailable, and there is no verified exact board FQBN. Do not invent a
board model. Once the existing Arduino ESP32 environment and ESP32Servo are
installed and the real board is selected, compile the patched original sketch:

```bash
arduino-cli compile --fqbn "$ESP32_FQBN" /Users/williamwu/Downloads/NatHack_Robot
```

`ESP32_FQBN` must be the physically verified board identifier, not a guessed value.
No upload command is executed or required for software testing. Host clang++
syntax-checks the staged .ino with explicit test stubs and compiles/runs the actual
patched Protocol.cpp/CommunicationSafety.cpp in tests. This verifies parser/state
logic, **not** ESP32 toolchain/API compatibility, electrical behavior or timing.
The firmware patch must be reviewed, board-built and deliberately installed before
its safety extensions exist on the real controller.

## Verification and module boundaries

Tests cover ASCII grammar/framing, partial/multiple TCP packets, invalid encoding,
oversized tails, exact TX/RX logging and exclusive files, lifecycle/ACK/DONE
separation, non-replay after timeout, heartbeat failure, controller-side all-actuator
watchdog, legacy compatibility, manual watchdog, raw-PWM limits, STOP/ESTOP/reset,
priority interruption, reconnect latch/disarm, physical gates, ground/wall sequence
simulation, pump/sponge/lift state, native C++ parser parity, and existing modules.

Full suite: `.venv/bin/python -m pytest -q` — **703 passed**, **1 existing legacy
floor-only warning**, 14.94 seconds. Tests that bind TCP ports require permission
outside this sandbox; all bind/connect targets are localhost.
The 66 Module 07 tests include tests of the real patched C++ parser, compiled using
host stubs; none contact a physical controller. No runtime dependencies were added.

Files created: `robot_comm/{__init__,__main__,protocol,logging,client,simulator,cli}.py`,
`tests/test_robot_communication.py`, `tests/firmware_stubs/*`, staged patch files,
`tools/generate_communication_demo.py`, actual local transcript artifacts, this report.
`main.py` adds dispatch and `README.md` documents usage. Modules 01–05 algorithms,
calibration, legal background and robot hardware settings are preserved. No Module
06, robot navigation or plan execution was added.

Future Module 06 has a reusable transport API but is **not ready for autonomous
hardware operation** merely because simulation passes. Required remaining physical
checks: board/FQBN/core/library build; real wiring/driver polarity and limits;
STOP and servo home mechanics; loss of Wi-Fi and silent heartbeat under load;
450/1000 ms controller watchdogs; ESTOP latch across TCP reconnect and explicit
reset with no restart; manual PWM direction/limit calibration; pump isolation;
servo travel; physical emergency stop; actual command latency. Localization and
heading calibration remain separate from communication, and UNVERIFIED pose never
arms the client.
