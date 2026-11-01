# Workspace setup

## Environment

The source targets Ubuntu 22.04 and ROS 2 Humble. The matching team workspace supplies:

- `kaiev26_msgs` and the ROS message packages used by the nodes.
- `kaiev26_maps`, route/map configuration, and GNSS/IMU localization.
- `pv26_perception` for the competition and `cone_perception` for cone-course brake inspection.
- Control command handling, actuator feedback, and vehicle drivers.
- The Gazebo bringup and simulated vehicle for simulation work.
- Detection engines, sensor calibration, and TF data appropriate to the vehicle setup.

## Layout and build

Place this repository at `~/KAI_ws/src/Decision` alongside the matching external packages:

```text
KAI_ws/
└── src/
    ├── Decision/
    │   ├── kaiev26_decision/
    │   ├── kaiev26_motion_control/
    │   ├── kaiev26_decision_fox/
    │   └── kaiev26_aeb/
    └── ...
```

```bash
cd ~/KAI_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  kaiev26_decision kaiev26_motion_control kaiev26_decision_fox kaiev26_aeb
source install/setup.bash
```

The [runbook](DECISION_RUNBOOK.md) describes the full workspace sequence. The [startup comparison](STARTUP_BEHAVIOR.md) describes the PAUSE/RUN behavior in the collected `kgm` revision.

## Offline checks

With NumPy and pytest installed, the following existing tests exercise controller geometry, the input-loss latch, and the visualization trail buffer:

```bash
PYTHONPATH="$PWD/kaiev26_aeb:$PWD/kaiev26_motion_control:$PWD/kaiev26_decision_fox${PYTHONPATH:+:$PYTHONPATH}" \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q \
  kaiev26_aeb/test/test_pursuit.py \
  kaiev26_motion_control/test/test_input_loss.py \
  kaiev26_decision_fox/test/test_trail.py
```

[VALIDATION.md](VALIDATION.md) records the collection checks. ROS message-dependent tests use the matching ROS workspace described above.
