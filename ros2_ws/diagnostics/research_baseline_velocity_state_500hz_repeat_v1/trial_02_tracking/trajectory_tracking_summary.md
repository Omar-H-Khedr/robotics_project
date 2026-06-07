# Trajectory Tracking Summary

- state_topic: `/joint_trajectory_controller/controller_state`
- command_topic: `/joint_trajectory_controller/joint_trajectory`
- joint_state_topic: `/joint_states`
- observed_commands: `5`
- jtc_state_samples: `40746`
- jtc_state_records: `40746`
- samples: `39837`
- duration_s: `79.695`
- joints: `joint_1, joint_2, joint_3, joint_4, joint_5, joint_6`
- max_abs_position_error_rad: `0.123190`
- mean_max_abs_position_error_rad: `0.004578`
- p95_max_abs_position_error_rad: `0.006777`
- mean_rms_position_error_rad: `0.002232`
- final_max_abs_position_error_rad: `0.001074`
- controller_state_duration_s: `81.539`
- controller_state_joints: `joint_1, joint_2, joint_3, joint_4, joint_5, joint_6`
- controller_state_max_abs_position_error_rad: `0.008861`
- controller_state_mean_max_abs_position_error_rad: `0.002747`
- controller_state_p95_max_abs_position_error_rad: `0.005861`
- controller_state_mean_rms_position_error_rad: `0.001392`
- controller_state_final_max_abs_position_error_rad: `0.000927`

This observer is passive. It does not publish commands or alter controller behavior.
