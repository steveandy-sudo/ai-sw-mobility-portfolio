# Experiment record

## Current record

Updated from Junghun Hwang's project report on 2026-09-17.

| Stage | Recorded status |
| --- | --- |
| Gazebo | Closed-loop validation of the AEB pipeline reported |
| Initial vehicle trials | Approximately 7 km/h; steering oscillation observed |
| Control-side update | The control team identified and corrected a steering-motor delay issue |
| Post-correction decision integration | The decision code has not yet been run with the correction |
| Competition result | No new result reported |

The control-side correction and successful integrated vehicle behavior are separate milestones. The current record establishes the former; the latter requires a new trial.

## Current source capabilities

The selected source provides route/zone mission handling; Pure Pursuit, Stanley and mixed tracking modes; cone-course perception/tracking combinations; and a Foxglove monitor. These describe available implementations. Configuration values, unit-test inputs and screenshot readouts are not aggregate vehicle-performance results.

## Next experiment: steering response and path tracking

**Question:** After the control-side correction, does the decision stack track the same route without the earlier steering oscillation?

This is a proposed protocol, not a completed experiment. A simulation or existing-log replay can prepare the analysis; validation of the physical correction requires a separately recorded vehicle trial.

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

The first useful artifact is one synchronized plot of target steering, actual steering, speed, and lateral error, followed by a concise explanation of any mismatch. Numeric results will be added when the corresponding raw record is available.

## Included visual material

The imported `foxglove_replay/` images illustrate the monitor during recorded-run inspection. They are used as interface examples; no new performance metric is inferred from them.
