#!/usr/bin/env bash
# collect_multi_trial_data.sh - Run N peg-in-hole trials with perception logging.
# Each trial gets its own perception log directory under OUTPUT_BASE.
# Usage: bash collect_multi_trial_data.sh [NUM_TRIALS] [OUTPUT_BASE]

set -euo pipefail

NUM_TRIALS="${1:-10}"
OUTPUT_BASE="${2:-diagnostics/multi_trial_dataset_v2}"
ROS_SETUP="/home/omar/code/robotics_project/ros2_ws/install/setup.bash"

LAUNCH_ARGS=(
    "use_gui:=false"
    "control_rate:=25.0"
    "position_gain:=3000.0"
    "position_derivative_gain:=10.0"
    "joint_damping_scale:=10.0"
    "inject_velocity_state:=true"
    "velocity_state_controller_config_path:=config/research_baseline_velocity_state_500hz.yaml"
    "search_recenter_duration_s:=8.0"
    "search_settle_duration_s:=9.0"
    "insert_handoff_timeout_s:=12.0"
    "search_entry_threshold_m:=0.0"
    "enable_perception_logging:=true"
)

mkdir -p "$OUTPUT_BASE"

for i in $(seq 1 "$NUM_TRIALS"); do
    TRIAL_DIR="${OUTPUT_BASE}/trial_$(printf '%02d' "$i")"
    mkdir -p "$TRIAL_DIR"
    echo "=== Trial $i/$NUM_TRIALS ==="
    
    source "$ROS_SETUP"
    ros2 launch thesis_bringup research_baseline.launch.py \
        "${LAUNCH_ARGS[@]}" \
        perception_log_dir:="$TRIAL_DIR" &
    LAUNCH_PID=$!
    
    # Wait for launch to finish (exit_on_done=True means it exits after one trial)
    wait $LAUNCH_PID || true
    
    # Move perception log if it exists
    if [ -f "$TRIAL_DIR/multimodal_observation_log.csv" ]; then
        ROWS=$(wc -l < "$TRIAL_DIR/multimodal_observation_log.csv")
        echo "Trial $i: $((ROWS - 1)) data rows"
    else
        echo "Trial $i: NO perception log found"
    fi
    
    # Brief cooldown between trials
    sleep 3
done

echo "=== All $NUM_TRIALS trials complete ==="
echo "Data in: $OUTPUT_BASE/"
