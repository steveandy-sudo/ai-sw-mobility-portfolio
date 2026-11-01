# Collection validation

Source: `TeamKAI-DL/Decision`, branch `kgm`, commit `203cd6ff024486171ac4cd97bdf82afacd1f6c73`.

## Completed checks

| Check | Result |
| --- | --- |
| Source file hashes | All 206 imported files match their source Git blobs and SHA-256 entries |
| Python syntax | 113 files parsed successfully |
| ROS package XML | 4 manifests parsed successfully |
| Existing offline tests | 33 passed |
| Curated Markdown links | Relative file and directory targets checked |

The offline test run used Python on Windows with NumPy and pytest. The checks exercised existing tests against the collected source.

## Existing offline tests

- [Pure Pursuit](../kaiev26_aeb/test/test_pursuit.py): lookahead geometry, steering limits, stale or invalid inputs, and a kinematic tracking exercise.
- [Input-loss latch](../kaiev26_motion_control/test/test_input_loss.py): input loss, latching, and the speed conditions for release.
- [Visualization trail](../kaiev26_decision_fox/test/test_trail.py): sampling, point limits, and trail reset.

The checks cover source integrity, syntax, and the ROS-independent logic above. ROS node, Foxglove bridge, Gazebo, and vehicle operation use the matching [team workspace](PORTFOLIO_SETUP.md). Competition outcomes and driving experiments are recorded separately in [EXPERIMENTS.md](EXPERIMENTS.md).
