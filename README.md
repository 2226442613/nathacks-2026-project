# BioGuard

### natHacks 2026 · Work in progress

BioGuard is a robotic cleaning prototype being developed for natHacks 2026. This repository brings together the embedded robot code, computer-side tools, and project documentation.

> This is a temporary README. Setup instructions, hardware details, results, and demo media will be added as the project is consolidated.

## Project approach

The intended workflow combines computer-side target detection and navigation with an ESP32-based robot that controls motors, pumps, and servos for ground and wall cleaning. The supplied firmware includes separate cleaning sequences and selection between two cleaning-fluid pumps. Full system integration and cleaning performance have not been verified in this repository.

## Repository layout

| Folder | Contents |
| --- | --- |
| [firmware/NatHack_Robot](firmware/NatHack_Robot/) | Arduino sketch and embedded C++ modules |
| [scripts](scripts/) | PC communication test script and utilities |
| [docs](docs/) | Integration guide and project documentation |
| [hardware](hardware/) | Space for schematics, wiring, and bill of materials |
| [src](src/) | Space for computer-side application code |
| [tests](tests/) | Space for automated tests and validation |
| [data/samples](data/samples/) | Space for synthetic or anonymized sample data |
| [config](config/) | Configuration examples |
| [assets](assets/) | Space for screenshots, images, and demo media |

## Current status

- Initial folder structure is in place.
- Supplied robot firmware, PC test script, and integration guide have been uploaded.
- The firmware references `Protocol.h`, but the matching protocol header and implementation were not supplied. The sketch is incomplete and cannot currently be built as supplied.
- The supplied firmware identifies itself as V1, while the integration guide describes V2 SIMPLE. Commands and behavior need to be reconciled before integration.
- Compilation, flashing, physical operation, navigation, and cleaning effectiveness have not been validated during this repository setup.

See the [firmware notes](firmware/NatHack_Robot/README.md) for the known version differences.

## Development setup

The embedded code uses Arduino, ESP32 Wi-Fi support, and the ESP32Servo library. Keep `NatHack_Robot.ino` and its sibling C++ files together in the `NatHack_Robot` sketch folder.

Before attempting a build, obtain the missing protocol files and confirm the intended firmware version, board configuration, library versions, and pin assignments. Detailed setup and run instructions are pending.

The PC tool is available at [`scripts/pc_test.py`](scripts/pc_test.py). It connects to a real device and can send actuator commands; it is not an offline test suite.

## Next steps

- Add the matching protocol source and reconcile firmware documentation.
- Add computer-side vision, navigation, and interface code.
- Document the hardware, wiring, dependencies, and calibration.
- Record validation results and add screenshots or a demo video.

## Contributors and license

Team credits and licensing details will be added later.
