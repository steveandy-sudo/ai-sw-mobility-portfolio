# Decision-Making and Vehicle Control — Team K.A.I.

**2026 Student AI·SW Mobility Competition · ROS 2 Humble**

[한국어](README.ko.md) · [Project portfolio](https://steveandy-sudo.github.io/projects/ai-sw-mobility/) · [Code guide](#code-guide) · [Setup](docs/PORTFOLIO_SETUP.md)

Route-based driving decisions, path tracking, and cone-course braking for an autonomous competition vehicle. This repository connects the implementations to the tools used to inspect mission states, target paths, steering commands, and vehicle feedback.

| Period | Role | Team | Environment |
| --- | --- | --- | --- |
| March 2026–ongoing | Junghun Hwang — Decision-making | 27 members; decision-making team of 5 | ROS 2 Humble · Gazebo · Foxglove |

## Results and current status

| Setting | Trial condition | Result |
| --- | --- | --- |
| Venue practice: preliminary course | Real vehicle, **10 m/s (36 km/h)** | Successful waypoint driving |
| Venue practice: final course | Real vehicle, **10 m/s (36 km/h)** | Successful waypoint driving |
| Venue practice: AEB command trial | **50 km/h**; competition requirement **30 km/h** | Stop command issued at the red cones; a brake hardware fault prevented a stop within the zone |
| Earlier Gazebo development | Closed-loop AEB trials | Pipeline integration exercised in simulation |
| Offline controller checks | [29 existing AEB tests passed](docs/VALIDATION.md) during repository preparation | Geometry, braking logic, trial profiles, and package layout checked |

The latest real-vehicle tests took place on **September 19–20, 2026**, during the organizer's open-practice period at the competition venue. Waypoint driving and AEB stopping were exercised repeatedly. These are practice results, not official competition results; the 50 km/h figure applies to AEB trials.

During footage review, two of the four brakes were reported inoperative. The vehicle overran the stopping zone, so this is a demonstrated stop-command event, not a successful physical stop.

For AEB, the success criterion was to detect the red cones, brake at the start of the red-cone zone, and stop before its end. The [experiment record](docs/EXPERIMENTS.md) separates these reported outcomes from the next measurements: stopping distance, path error, steering response, and repeatability.

## Venue-practice excerpts

[![Final-course waypoint driving](https://steveandy-sudo.github.io/media/projects/ai-sw-mobility/03-final-course-waypoint-poster.jpg)](https://steveandy-sudo.github.io/projects/ai-sw-mobility/#evidence-and-current-result)

**[Final-course waypoint GIF](https://steveandy-sudo.github.io/media/projects/ai-sw-mobility/03-final-course-waypoint.gif)** — 17 continuous seconds, at original speed; reported run speed 10 m/s (36 km/h).

**[50 km/h AEB trial GIF](https://steveandy-sudo.github.io/media/projects/ai-sw-mobility/04-aeb-braking-trial.gif)** — nine continuous seconds covering approach, the red-cone event, and the stopping-zone overrun. The stop command was issued, but two of four brakes were reported inoperative. This is not a successful in-zone stop.

Both excerpts are from September 20 venue practice. The [case study](https://steveandy-sudo.github.io/projects/ai-sw-mobility/#evidence-and-current-result) includes click-to-play GIFs and context. Speeds are participant-reported; raw telemetry is not overlaid.

## Development focus

- **Driving decisions:** route and zone context, mission-state handling, and target path/speed interfaces.
- **Path tracking:** Stanley control, steering bounds and rate limits, and integration with vehicle-command handling.
- **Cone-course AEB:** cone-boundary center paths, red-entry stop logic, and missing/stale-input handling.
- **Integration and observation:** Gazebo development, repeated real-vehicle venue practice, and Foxglove inspection of recorded and live signals.

## System architecture

The main driving test workflow is:

```mermaid
flowchart LR
    Inputs[Perception and localization] --> Decision[Route / zone context and mission FSMs]
    Decision -->|Target path and speed| Motion[Motion control]
    Feedback[Vehicle feedback] --> Motion
    Motion -->|Raw command| TUI[Test interface gate]
    TUI -->|Planning command| Governor[External Control command governor]
    Governor --> Vehicle[Vehicle]
    Vehicle --> Feedback
    Decision --> Monitor[Foxglove monitor]
    Feedback --> Monitor
```

In this workflow, the test interface receives `/decision/raw_command` and owns publication to `/planning/command`. The cone-course AEB package has a separate trial workflow. It provides camera–LiDAR fusion and camera-only perception, each paired with Pure Pursuit or Stanley tracking.

### Design choices visible in the code

| Choice | Implementation | Evaluation relevance |
| --- | --- | --- |
| Separate mission logic from tracking | Mission FSMs produce target path/speed; motion control produces steering and speed commands | State selection and tracking response can be inspected separately |
| Bound steering magnitude and change | Controller outputs pass through angle and rate constraints | Geometric commands must be compared with actuator response |
| Brake on unusable AEB inputs | Stale state/path, invalid geometry, or wrong path frame requests zero speed and braking | Input failures have explicit reasons in the output |
| Confirm and retain the red entry gate | Repeated observations confirm a gate; motion compensation updates it; crossing latches braking | A later cone row cannot silently replace the confirmed stop gate |

The braking branch retains the last limited steering command while requesting a stop. These are implementation properties; measured stopping distance and tracking accuracy require logged trials.

## Code guide

| Area | Start here |
| --- | --- |
| Route and mission decisions | [Route/zone manager](kaiev26_decision/kaiev26_decision/route_zone_manager_node.py) · [Planning engine](kaiev26_decision/kaiev26_decision/main_planning_engine_node.py) · [Mission FSMs](kaiev26_decision/kaiev26_decision/scenario_modules/) |
| Path tracking | [Motion controller](kaiev26_motion_control/kaiev26_motion_control/motion_control_node.py) · [Configuration](kaiev26_motion_control/config/motion_control.yaml) |
| Cone geometry | [Fusion geometry](kaiev26_aeb/kaiev26_aeb/core.py) · [Camera-only geometry](kaiev26_aeb/kaiev26_aeb/yolo_core.py) |
| AEB tracking and stops | [Pure Pursuit / input checks / red stop](kaiev26_aeb/kaiev26_aeb/pursuit.py) · [Stanley](kaiev26_aeb/kaiev26_aeb/stanley.py) · [Trial profiles](kaiev26_aeb/kaiev26_aeb/trial_profiles.py) |
| Trial operation and monitoring | [Decision test interface](kaiev26_decision/kaiev26_decision/test_tui.py) · [AEB test interface](kaiev26_aeb/kaiev26_aeb/trial_tui.py) · [Foxglove package](kaiev26_decision_fox/README.md) |

## Recorded-run inspection

[AEB cone and command replay (GIF)](https://steveandy-sudo.github.io/media/projects/ai-sw-mobility/05-aeb-foxglove-replay.gif) shows cone geometry, target speed, and brake-command plots in Foxglove. The screen recording retains the visible 2× replay setting. Target speed is not measured vehicle speed; this view is presented separately from the onboard trial excerpt.

<img src="docs/foxglove_replay/replay_mid.png" alt="Foxglove MCAP replay with route context, cameras, mission states, and target versus actual vehicle signals" width="900" />

*Existing MCAP replay view: route context, camera input, state readiness, and target/actual signals in one monitor. This frame shows the interface in a waiting state; its instantaneous readouts are not a completed-run performance summary.*

## Build and documentation

Use Ubuntu 22.04 / ROS 2 Humble and the matching team workspace, including `kaiev26_msgs`, Control, Localization, perception, and the sensor/simulator dependencies for the selected workflow. Camera-based AEB requires external model weights. Place this repository at `~/KAI_ws/src/Decision`, then:

```bash
cd ~/KAI_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  kaiev26_decision kaiev26_motion_control kaiev26_decision_fox kaiev26_aeb
source install/setup.bash
```

This is the intended ROS build workflow. Preparation checks were offline; the full ROS build and integrated vehicle run require the team environment.

- **Run or inspect:** [Setup](docs/PORTFOLIO_SETUP.md) · [Runbook](docs/DECISION_RUNBOOK.md) · [Mission scenarios](docs/MISSION_SCENARIOS.md)
- **Understand the design:** [Architecture](docs/ARCHITECTURE.md) · [Route tracking](docs/ROUTE_TRACKING.md) · [Parameters](docs/PARAMETER_BOOK.md)
- **Trace the evidence:** [Experiments](docs/EXPERIMENTS.md) · [Validation](docs/VALIDATION.md) · [Source snapshot](docs/SOURCE_MAP.md)
