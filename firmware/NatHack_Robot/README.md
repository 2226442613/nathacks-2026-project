# NatHack Robot firmware

Imported from the supplied Hackathon Files folder without source changes.

Open NatHack_Robot.ino with its sibling .cpp and .h files in the same Arduino sketch folder. The code uses Arduino ESP32 WiFi and ESP32Servo.

## Incomplete source set

NatHack_Robot.ino and Cleaner.cpp include Protocol.h, but Protocol.h and its implementation were not supplied. The sketch cannot be built as supplied; obtain the matching protocol source before building.

## Version mismatch

The supplied sketch announces NatHack Robot V1.0 and CONNECTED,NATHACK_V1. Config.h sets CLEAN_DRIVE_PWM to 110 and servo home angles to 0 degrees. The guide in docs/NatHack_电脑端对接指导.md describes V2 SIMPLE, duration-based MOVE commands, PWM 170, and home angles of 180 degrees. Do not assume that guide matches this source set. scripts/pc_test.py accompanies the older source; MOVE semantics cannot be verified without the missing protocol implementation.

Wi-Fi values in Config.h are placeholders. Keep real credentials out of GitHub. No compilation, flashing, device connection, or physical testing was performed during this import.
