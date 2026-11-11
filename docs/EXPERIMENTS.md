# Competition and experiment record

## Official competition

Team K.A.I. completed all required missions in both the preliminary and final rounds of the 2026 Student AI·SW Mobility Competition under the competition rules.

| Category | Division | Team result |
| --- | --- | --- |
| Self-built Autonomous Vehicle | Autonomous Driving Implementation | Grand Prize — Minister of Land, Infrastructure and Transport Award |
| Self-built Autonomous Vehicle | Creative Technology | Special Award for the vehicle technical report |

The participant supplied the competition outcome and award details. The [organizer's award guide](https://carsa.kr/s1/s1_4.php) identifies the award categories.

## Test Week and field operation

Junghun Hwang collected vehicle logs, analyzed them in Foxglove, revised the planning logic, and ran follow-up vehicle tests.

### October 17 K-City final-course tests

| Recording | `FORCED_S_CONTINUE` entries | Subsequent state |
| --- | --- | --- |
| `20261017_114251_414559_decision_case_03_final_all_v7` | 5 | |
| `20261017_171607_574849_decision_case_03_final_all_v7` | 3 | `RETURN` |

The log review covers S-shaped fallback path selection and publication and the state transition above. Physical contact remains unconfirmed.

### Cached-path reprojection

Near 15:08:46 in `20261017_145703_857137_decision_case_03_final_all_v10`, the vehicle moved about 3.07 m in 0.576 s. The cached path timestamp advanced while its coordinates remained in the earlier vehicle frame. Hwang implemented reprojection from the saved and current odometry poses and preserved the original timestamp when the required pose was unavailable (`b71f307`). The mismatch's role in forced-path selection remains open.

### Offline comparison and temporary horizon setting

From October 21 onward, a condition could exclude the second obstacle row from collision checks if the 35 m local path could not include the row and the 12 m return target. The recorded geometry required about 40.4–41.3 m. The offline comparison record (E05) uses the same three October 17 recordings above and changes only that exclusion condition. At 10 ms intervals, the first path accounting for both rows appeared 1.10–1.23 s earlier. Attribution for executing this comparison is pending.

Hwang separately committed a provisional 35 to 45 m local-path setting with the 12 m return target retained (`2eb64af`). The 45 m recalculation record (E11) still shows geometric overlap between some candidates and observed obstacles. Both changes are included in team branches `main` and `kgm`. The [validation record](VALIDATION.md) describes the associated configuration and avoidance checks.

### October 23 team vehicle retests

The team Git record (`a22188e`) reports passes in Midan trials `1452` and `1456` after combined changes to perception filtering, forced-path replanning, Stanley-only tracking during avoidance, and avoidance speed settings. The original afternoon rosbags are pending review. The different course and configuration leave the effect of the 45 m setting alone unconfirmed.

## Earlier venue practice

| Trial | Reported outcome |
| --- | --- |
| Preliminary-course waypoint driving | Reached up to 11 m/s (39.6 km/h) |
| Final-course waypoint driving | Reached up to 11 m/s (39.6 km/h) |
| Red-cone command trial, approach at 50 km/h | Red-cone recognition triggered the stop request at the stopping-zone entrance |

The waypoint speeds refer to venue practice. The AEB excerpt records recognition and command triggering. The [project portfolio](https://steveandy-sudo.github.io/projects/ai-sw-mobility/#real-vehicle-trials) presents the corresponding onboard and Foxglove excerpts.

## Initial controller comparison

The vehicle technical report describes a Pure Pursuit–Stanley versus Stanley-only comparison on matched Gazebo course segments at 6 m/s in Section 4.3.1 (pp. 239–240) and its summary table (p. 256). The heading-error p95 and distance-normalized steering-command variation results are pending review of the original runs and metric calculation.
