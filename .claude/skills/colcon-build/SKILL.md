---
name: build
description: Build the ROS2 workspace with colcon. Use after making code changes.
---

Build all packages from the workspace root:

```bash
cd /home/bwave/bwave/robot_follow && colcon build
```

To build only specific packages, use `--packages-select`:

```bash
cd /home/bwave/bwave/robot_follow && colcon build --packages-select robot_follow
```

After a successful build, remind the user to source:

```bash
source /home/bwave/bwave/robot_follow/install/setup.bash
```

If the build fails, show the error output and suggest fixes. Common issues:
- Missing ROS2 environment: `source /opt/ros/humble/setup.bash` first
- agibot arch detection failure on unrecognized CPU architecture
- C++17 features used in packages that compile as C++14
