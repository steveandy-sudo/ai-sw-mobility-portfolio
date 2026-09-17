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

## Next integration record

For the next decision-stack trial, record:

1. Decision, Control and Localization versions; controller and parameter file.
2. Test environment, course, speed and initial pose.
3. Target versus actual steering response after the motor-delay correction.
4. Path-tracking behavior and whether the earlier oscillation remains.
5. Stop-state transitions and observed vehicle stopping behavior.
6. MCAP/log location and a matching video or monitor capture, if available.

Use the trial result to update this table and the portfolio page. Keep the raw record available so observations can be traced to the corresponding settings.

## Included visual material

The imported `foxglove_replay/` images illustrate the monitor during recorded-run inspection. They are used as interface examples; no new performance metric is inferred from them.
