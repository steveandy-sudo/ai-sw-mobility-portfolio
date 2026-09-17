# Collection validation

Source: `TeamKAI-DL/Decision`, branch `hwj`, commit `11cb96570c5cb8b303f0ca673378e5293f76a181`.

## Completed checks

| Check | Result |
| --- | --- |
| Imported Git blobs | 144 files collected with byte-preserving source mapping |
| Python syntax | 81 files parsed successfully |
| ROS package XML | 4 manifests parsed successfully |
| Existing offline AEB tests | 29 passed |
| Local links in added documentation | 58 file and directory targets verified |

The offline run used Python 3.12 on Windows with NumPy, pytest 9.1.1 and PyYAML 6.0.3. Dependencies were installed in an isolated local check directory; source code was not altered to make the tests pass.

Imported source formatting is retained byte-for-byte, including existing trailing spaces in upstream Markdown. Whitespace checks for the newly authored documents pass.

Test files:

- `kaiev26_aeb/test/test_pursuit.py`
- `kaiev26_aeb/test/test_trial_profiles.py`
- `kaiev26_aeb/test/test_package_layout.py`

Coverage includes Pure Pursuit geometry, invalid/stale input braking, a kinematic tracking exercise, the scenario matrix and package/parameter layout. The test's 30 km/h input is a synthetic controller test condition, not a measured vehicle speed.

## Checks requiring other environments

- ROS 2 message-dependent tests and full `colcon build`/`colcon test`.
- Foxglove extension build and live bridge integration.
- Gazebo integration with the external vehicle workspace.
- Real-vehicle execution of the decision stack after the reported Control correction.

No vehicle or hardware interface was started during collection. Reported project experiments are documented separately in [EXPERIMENTS.md](EXPERIMENTS.md).
