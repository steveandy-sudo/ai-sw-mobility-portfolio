# Setup and execution scope

## Environment

The source targets Ubuntu 22.04 and ROS 2 Humble. A complete team workspace supplies dependencies beyond this repository:

- `kaiev26_msgs` and the relevant ROS message packages.
- Control command handling and vehicle feedback.
- Localization and perception for the selected driving mode.
- Camera drivers, calibration/TF and, for fusion trials, the installed Ouster driver.
- The Gazebo bringup and simulated vehicle for simulation.
- A compatible cone-detection model for camera-based AEB; model weights are not included in the source snapshot.

Use the dependencies for the selected `hwj` version. The existing [runbook](DECISION_RUNBOOK.md) and package manifests specify interfaces; this collection does not invent version pins for external repositories.

## Workspace layout

The original tools refer to `~/KAI_ws/src/Decision`. Keep that directory name when placing this collection in the team workspace:

```text
KAI_ws/
└── src/
    ├── Decision/                 This repository
    │   ├── kaiev26_decision/
    │   ├── kaiev26_motion_control/
    │   ├── kaiev26_decision_fox/
    │   └── kaiev26_aeb/
    └── ...                       Required external team packages
```

## Build in the prepared ROS environment

```bash
cd ~/KAI_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  kaiev26_decision kaiev26_motion_control kaiev26_decision_fox kaiev26_aeb
source install/setup.bash
```

This command is retained from the source instructions. A ROS build was not performed on the Windows preparation host.

## Offline AEB checks

From this repository root, with NumPy, PyYAML and pytest available:

```bash
PYTHONPATH="$PWD/kaiev26_aeb${PYTHONPATH:+:$PYTHONPATH}" python -m pytest -q \
  kaiev26_aeb/test/test_pursuit.py \
  kaiev26_aeb/test/test_trial_profiles.py \
  kaiev26_aeb/test/test_package_layout.py
```

These existing tests exercise controller logic, trial combinations and package configuration without connecting to ROS or a vehicle. See [validation results](VALIDATION.md).

## Simulation and vehicle operation

Follow the versioned [runbook](DECISION_RUNBOOK.md) and [AEB package instructions](../kaiev26_aeb/README.md). The main decision pipeline, AEB and SysID are separate command-producing workflows. Preserve the original command-ownership, MANUAL/AUTO and E-Stop procedures when preparing a trial.

The [Foxglove package guide](../kaiev26_decision_fox/README.md) covers the read-only monitor and its separate extension build. No Foxglove extension build, ROS launch or hardware command was executed while preparing this repository.
