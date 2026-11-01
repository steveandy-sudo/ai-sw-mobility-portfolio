# E-stop-controlled departure

The participant identified the source by its departure procedure: release the E-stop to start driving. The `kgm` head matches the PAUSE/RUN departure change.

| Revision | Startup handling |
| --- | --- |
| `main` at `c013aefadb60eea63b89bedb845661d8d0e23f6a` | Readiness requires E-stop release; AUTO starts a three-second TUI countdown |
| `kgm` at `203cd6ff024486171ac4cd97bdf82afacd1f6c73` | Readiness allows E-stop PAUSE; AUTO enters RUNNING; the vehicle waits under the physical PAUSE interlock until RUN |

The `kgm` commit is the direct successor of the compared `main` revision. Its changes are concentrated in readiness, the test TUI, and their tests.

## Code locations

- [Readiness](../kaiev26_decision/kaiev26_decision/test_readiness.py): E-stop PAUSE is reported while communication, vehicle feedback, and the other readiness conditions are evaluated.
- [Test TUI](../kaiev26_decision/kaiev26_decision/test_tui.py): `WAIT_AUTO` transitions directly to `RUNNING` on AUTO. `track_estop` records PAUSE/RUN changes.
- [TUI tests](../kaiev26_decision/test/test_decision_test_tui.py): `test_rulebook_start_waits_in_estop_pause_and_goes_on_run` covers this transition sequence.
- [Readiness tests](../kaiev26_decision/test/test_gnss_route_odometry.py): `test_estop_pause_does_not_fail_vehicle_or_steering_readiness` covers readiness during PAUSE.

The TUI state describes the software run. The external vehicle-control system owns physical braking and actuator interlocks. MANUAL/AUTO and E-stop operation follow the matching team workspace and vehicle procedures.
