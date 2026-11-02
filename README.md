# Thermal Rover

I'm building Thermal Rover to see how much a small rover can learn about a room
as it moves through it. An LD19 LiDAR maps the space, an IMU helps track motion,
and an MLX90640 thermal camera adds temperature readings to the surfaces the
rover sees. A Windows app turns those streams into a map I can build while
driving, then save and replay.

## What it does

- Builds a live 2D map from LiDAR scans and overlays thermal readings on observed surfaces.
- Drives with WASD, an adjustable speed limit, and controls for measured distance and turns.
- Saves sensor packets and events in SQLite so a run can be replayed, scrubbed through, and exported as a PNG map.
- Shows sensor health, connection status, packet statistics, and the latest thermal image in a Diagnostics workspace.
- Offers three app themes: Polar, Paper, and Graphite.

The app has **Map & capture**, **Drive**, and **Diagnostics** workspaces. It starts with an empty map; connect a rover and start recording to build one, or open an existing session to explore it offline.

## How the rover maps

The ESP32-S3 reads the sensors, controls the motors, and sends UDP packets to the laptop. LiDAR and IMU data estimate the rover's movement on board, giving the motor controller feedback for measured moves. The desktop app aligns LiDAR scans into a local map and projects thermal readings onto nearby surfaces. It uses the same LiDAR/IMU estimation code when replaying recordings that contain raw sensor data.

The right wheel encoder on my rover failed during development. The current
system uses LiDAR and IMU feedback instead of wheel ticks for motion and
mapping. That change made scan matching and IMU heading central to the design.

The live app and firmware share protocol 3. Sessions are stored as raw packets and ordered events, which lets replay use the same data path as a live run.

## Build the Windows app

You'll need Windows, CMake 4.2 or newer, a Visual Studio C++ toolchain with C++23 support, and an OpenGL 4.6-capable driver. The current build uses Visual Studio 2026. CMake fetches GLFW and GLM; Dear ImGui is included as a Git submodule.

From PowerShell in the repository root:

```powershell
git submodule update --init --recursive
cmake -S . -B build -DBUILD_TESTING=OFF
cmake --build build --config Release --target ThermalRoverApp
& '.\build\app\Release\ThermalRoverApp.exe'
```

With a single-configuration generator such as Ninja, the executable is usually at `build/app/ThermalRoverApp.exe`. The build copies the app's font beside it; keep the `fonts` folder there. Close the app before rebuilding it on Windows.

## Run it with the rover

The firmware is an ESP-IDF 6.0.2 project in `hardware/` for an Adafruit Metro ESP32-S3. The rover uses an LD19 LiDAR, MLX90640 thermal camera, MPU-6050 IMU, MAX17048 battery monitor, and TB6612FNG motor driver. Pin assignments are in `protocol/include/protocol/robot_config.hpp` and `hardware/main/src/hardware.cpp`.

Build from an ESP-IDF terminal:

```powershell
Set-Location hardware
idf.py build
```

Flash with `idf.py -p COM3 flash monitor`, replacing `COM3` with the board's serial port. On first boot, enter `WIFI|<ssid>|<password>` over USB serial when prompted. The rover stores those credentials in flash.

With the rover and laptop on the same network, open **Diagnostics** and use **Link rover** to discover it. You can also enter its IPv4 address and port directly. Start a recording in **Map & capture** to build a map. In **Drive**, arm the rover before using W/S to move and A/D to turn; release the keys or use **Emergency stop** to stop. The default speed limit is 30%.

## Where the project is now

I've used the assembled rover for manual movement and live LiDAR and thermal mapping. A complete physical session has also been replayed and exported offline. I'm continuing to improve map consistency and measure how accurately the rover drives set distances and angles. Autonomous exploration is the longer-term direction.

## Repository layout

- `app/`: Windows app, controls, and SQLite sessions
- `core/`: sensor decoding, mapping, rendering, and desktop networking
- `protocol/`: shared packet format, LiDAR/IMU estimation, and motion control
- `hardware/`: ESP-IDF firmware and sensor drivers
- `cad/` and `schematics/`: mechanical and circuit designs

Development note: I designed the rover's hardware architecture, defined the software requirements and system interactions, and conducted physical integration, testing, and debugging. I implemented the initial software foundation independently. Later development involved substantial AI coding assistance, with implementation guided by my specifications and evaluated through testing on the physical rover.

## License
Copyright © 2026 Talin Sharma. All rights reserved. See [License](LICENSE). Third-party libraries and the Melexis MLX90640 driver retain their own licenses.
