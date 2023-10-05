# Open-source Travel Photography Star Tracker

This is a in-progress project that aims to build an easy-to-use short duration, light-weight start tracker for astro photography.

Features:
- Roughly 15 minutes tracking time
- No manual alignment required
- Battery/USB powered
- Designed as in-place tripod adapter
  - 3/8" female on bottom side for direct mounting on a tripod
  - 3/8" male screw on top side for ball head attachment

## Project structure

- Raspberry Pi software
  - Main python application (in progress)
    - Star-based attitude estimation (in progress)
    - Trajectory calculation (in progress)
    - User interface/control (not started yet)
  - Raspberry Pi camera (device t.b.d.)
- PCB (not started yet)
  - Micro controller for precision
  - Battery and power management
  - Stepper motor drivers
- 3D printed mechanical parts (not started yet)
