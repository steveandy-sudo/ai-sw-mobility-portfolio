# Experiment record

## Current record

Updated from Junghun Hwang's project report on **2026-09-22**. The latest trials took place on **2026-09-19 and 2026-09-20**, during the organizer's open-practice period at the competition venue, using the real vehicle.

| Stage | Recorded status |
| --- | --- |
| Gazebo | Closed-loop validation of the AEB pipeline reported |
| Initial vehicle trials | Approximately 7 km/h; steering oscillation observed |
| Control-side update | The control team identified and corrected a steering-motor delay issue |
| Preliminary-course waypoint driving | Successful at **10 m/s (36 km/h)** during venue practice |
| Final-course waypoint driving | Successful at **10 m/s (36 km/h)** during venue practice |
| AEB command trial | At **50 km/h**, the stop command was issued at the red cones. Two of four brakes were reported inoperative; the vehicle overran the stopping zone. Competition requirement: **30 km/h** |
| Official competition result | The latest report concerns open practice, not official qualifying or final results |

**AEB success criterion:** recognize the red cones, apply braking at the start of the red-cone zone, and come to a complete stop before the end of that zone.

The tests were repeated over the two practice days. The participant recalled roughly ten repetitions, but did not provide an attempt-by-attempt log or a breakdown by course and speed. No exact trial count, success rate, or stopping-distance statistic is published from that recollection. The 50 km/h figure is the AEB trial speed, not a successful stopping-speed claim; waypoint driving was reported at 10 m/s (36 km/h).

The earlier steering-delay correction and the new course-driving and AEB command trials are recorded as development milestones. Quantifying the correction's effect requires compatible before/after logs; the latest report alone does not establish a reduction in steering error.

## Current source capabilities

The selected source provides route/zone mission handling; Pure Pursuit, Stanley and mixed tracking modes; cone-course perception/tracking combinations; and a Foxglove monitor. These describe available implementations. Configuration values, unit-test inputs and screenshot readouts are not aggregate vehicle-performance results.

## Follow-up analysis: steering response, path tracking, and AEB

**Question:** What path error, steering response, and stopping margin accompanied the venue-practice trials, and how consistently were those outcomes repeated?

Course driving succeeded; the AEB command was issued, but the physical stopping criterion was not achieved. This distinction was clarified during footage review on September 22. The quantitative analysis below is planned and requires the corresponding logs. Simulation or recorded-data replay should be labeled separately from the vehicle trial it represents.

### Keep the comparison interpretable

Record Decision, Control, and Localization commits; controller mode; the full parameter file; route; initial pose; speed profile; environment; and intervention times. Compare runs only when these conditions are compatible. If no matching pre-correction log exists, report the new run on its own rather than inventing a before/after comparison.

| Measure | Definition and unit | Record needed |
| --- | --- | --- |
| Steering tracking error | Commanded minus measured steering on aligned timestamps; MAE and maximum absolute error in degrees | Command source, vehicle steering feedback, timestamp convention, unit conversion |
| Response delay | A specified command change to a specified measured-response threshold, in milliseconds | Command/feedback traces and the chosen threshold; report each event |
| Lateral tracking error | Signed distance to the reference path; RMS and maximum absolute value in meters | Localization, reference path, common frame, and time interval |
| Completion | Completed runs / attempted runs, with interventions and stop reasons listed | A definition of completion and every attempt, including interrupted runs |
| AEB stop behavior | Stop trigger, first brake request, zero-speed time, and signed final position relative to the stop gate | Cone/gate estimate, commands, pose, vehicle speed, and clock alignment |

In the decision-test workflow, distinguish `/decision/raw_command`, `/planning/command`, and the Control output `/vehicle/command`; they are different points in the command chain. Read actual fields and units from the matching message definitions. Use `/vehicle/state`, `/localization/odometry`, `/planning/target_path`, and route/mission context when present. A topic absent from a log should be listed as missing rather than estimated from a screenshot.

### Retain a traceable trial record

For each attempt, keep a run ID, date, source versions, settings, MCAP/log path, analyzed start/end timestamps, exclusions, computed result table, and a matching video or monitor capture. A short portfolio clip can illustrate the behavior; metrics must state whether they cover the full attempt or only a selected interval.

Final-course and AEB onboard excerpts are now available on the project page. The AEB excerpt includes the stopping-zone overrun. Preliminary-course footage and raw speed/command records are still needed. Path-error and stopping-distance statistics will be added when the corresponding raw record is available.

The collected source is the September 15 `hwj` snapshot documented in [SOURCE_MAP.md](SOURCE_MAP.md). The exact Decision, Control, and Localization revisions used on September 19–20 have not yet been supplied; the performance report does not establish that the archived snapshot is identical to the tested workspace.

## Included visual material

The imported `foxglove_replay/` images illustrate the monitor during recorded-run inspection. They are used as interface examples; no new performance metric is inferred from them.
