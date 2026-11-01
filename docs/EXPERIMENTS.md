# Competition and experiment record

## Official competition

Team K.A.I. completed all required missions in both the preliminary and final rounds of the 2026 Student AI·SW Mobility Competition under the competition rules.

| Category | Division | Team result |
| --- | --- | --- |
| Self-built Autonomous Vehicle | Autonomous Driving Implementation | Grand Prize — Minister of Land, Infrastructure and Transport Award |
| Self-built Autonomous Vehicle | Creative Technology | Special Award for the vehicle technical report |

The participant supplied the competition outcome and award details. The [organizer's award guide](https://carsa.kr/s1/s1_4.php) identifies the award categories.

## Test Week and field operation

Junghun Hwang's main role during Test Week was vehicle operation for experimental runs. His obstacle-avoidance field work covered approach deceleration, collision-free passage, return to the reference route, and acceleration afterward. Foxglove log analysis and discussions with teammates informed corrections between trials.

## Earlier venue practice

| Trial | Reported outcome |
| --- | --- |
| Preliminary-course waypoint driving | Reached up to 11 m/s (39.6 km/h) |
| Final-course waypoint driving | Reached up to 11 m/s (39.6 km/h) |
| Red-cone command trial, approach at 50 km/h | Red-cone recognition triggered the stop request at the stopping-zone entrance |

The waypoint speeds refer to venue practice. The AEB excerpt records recognition and command triggering. The [project portfolio](https://steveandy-sudo.github.io/projects/ai-sw-mobility/#real-vehicle-trials) presents the corresponding onboard and Foxglove excerpts.

## Initial controller comparison

The initial Pure Pursuit–Stanley implementation was compared with Stanley alone on matched Gazebo course segments at 6 m/s:

| Metric | Initial hybrid result relative to Stanley |
| --- | --- |
| Heading-error p95 | Approximately 14% lower |
| Steering-command total variation | Approximately 10% lower |

These figures describe the initial simulation comparison. They are recorded separately from venue-practice speeds and official competition results.
