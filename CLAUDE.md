# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

```bash
colcon build          # from workspace root (no src/ subdir — packages sit at root)
source install/setup.bash
```

Packages are discovered via `package.xml`, not a top-level CMakeLists.txt.

## Branch naming

`feature/<name>` branches, PR to `main`.

## Gotchas

- **No tests exist.** Only `ament_lint` checks, and only for `lidar_pkg` and `robot_follow` packages.
- **C++ standards vary**: `robot_follow` = C++17, `agibot` and `uwb_serial_pub` = C++14, `lidar_pkg` = default. Avoid C++17-only features in shared headers.
- **agibot arch detection**: `CMakeLists.txt` matches `CMAKE_HOST_SYSTEM_PROCESSOR` against `x86_64|amd64|aarch64|arm64` and `FATAL_ERROR`s on mismatch. SDK `.so` files are vendored in `agibot/lib/`.
- **`robot_follow/launch/d1_bringup.launch.py`** references non-existent package `d1_bringup` — don't use it.
- **`httplib.h`** is vendored at `robot_follow/include/httplib.h` (header-only HTTP/WebSocket).
- Custom interfaces live in `robot_follow/msg/` — if you add a `.msg` file, add it to `CMakeLists.txt`'s `rosidl_generate_interfaces`.
