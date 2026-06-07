# Final Side-Load Retry Repeat Validation Analysis

Date: 2026-06-08 local time

## Command

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run experiment_manager research_baseline_repeat_validator \
  --trials 5 \
  --timeout-s 820 \
  --output-dir diagnostics/research_baseline_final_sideload_retry_500hz_repeat_v2 \
  --per-trial-tracking-logs \
  --extra-launch-arg use_gui:=false \
  --extra-launch-arg control_rate:=25.0 \
  --extra-launch-arg position_gain:=3000.0 \
  --extra-launch-arg position_derivative_gain:=10.0 \
  --extra-launch-arg joint_damping_scale:=10.0 \
  --extra-launch-arg inject_velocity_state:=true \
  --extra-launch-arg velocity_state_controller_config_path:=config/research_baseline_velocity_state_500hz.yaml \
  --extra-launch-arg search_recenter_duration_s:=8.0 \
  --extra-launch-arg search_settle_duration_s:=9.0 \
  --extra-launch-arg insert_handoff_timeout_s:=12.0
```

## Result

- Trials completed: 5
- Physical successes: 5
- Success rate: 1.0
- Timeouts: 0
- Safety aborts: 0
- Side-load aborts: 0
- Force/XY safety thresholds were not relaxed.

| Trial | Outcome | Depth m | Final XY m | Max pre-depth XY m | Pre-depth recenters | Entry descents | Capture descents | Shallow withdrawals | Final side-load retries |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | SUCCESS | 0.0200 | 0.0002 | 0.0014 | 0 | 1 | 1 | 0 | 0 |
| 2 | SUCCESS | 0.0202 | 0.0007 | 0.0014 | 0 | 1 | 1 | 0 | 0 |
| 3 | SUCCESS | 0.0203 | 0.0006 | 0.0021 | 2 | 1 | 3 | 0 | 0 |
| 4 | SUCCESS | 0.0206 | 0.0008 | 0.0015 | 1 | 1 | 2 | 0 | 0 |
| 5 | SUCCESS | 0.0198 | 0.0002 | 0.0017 | 1 | 2 | 1 | 0 | 0 |

## Diagnosis

The preceding v1 candidate reached only 4/5 because Trial 5 aborted on
sustained side-load at full insertion depth before the final trajectory elapsed.
The retained source now allows the same bounded final/deep side-load
withdrawal-retry path to run from the sustained side-load detector when
meaningful depth and contact evidence are already present. In the retained v2
repeat set, that path did not trigger because all final descents ended inside
the physical clearance gate.

The original no-contact pre-depth drift blocker is reduced but not eliminated:
Trials 3, 4, and 5 still crossed the 1 mm gate before meaningful depth and
required bounded recenter recovery. Trial 3 used both allowed recenters and
still succeeded, which is useful stress evidence, but the behavior is close to
the configured recenter budget.

SEARCH-entering robustness is not solved by this retained pass. All v2 trials
bypassed SEARCH because APPROACH ended inside the fixed physical clearance.
The prior v1 candidate contained one SEARCH -> INSERT physical success, but a
dedicated SEARCH-entered repeat set is still needed.

## Limitations

- This is a 5-trial simulated repeat pass, not final autonomous peg-in-hole
  validation.
- A 10-trial extension was not run after the multiple candidate passes in this
  milestone.
- The final/deep side-load retry is implemented and fail-closed, but the
  retained 5-trial pass did not exercise it.
- Gazebo contact-topic positives remain unreliable in this line of work; the
  retained contact evidence is task-side wrench-derived contact.
