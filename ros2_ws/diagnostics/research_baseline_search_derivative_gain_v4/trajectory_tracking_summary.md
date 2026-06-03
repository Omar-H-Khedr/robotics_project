# Trajectory Tracking Summary

- state_topic: `/joint_trajectory_controller/controller_state`
- command_topic: `/joint_trajectory_controller/joint_trajectory`
- joint_state_topic: `/joint_states`
- observed_commands: `10`
- jtc_state_samples: `50345`
- jtc_state_records: `50345`
- samples: `49984`
- duration_s: `199.939`
- joints: `joint_1, joint_2, joint_3, joint_4, joint_5, joint_6`
- max_abs_position_error_rad: `0.140279`
- mean_max_abs_position_error_rad: `0.007868`
- p95_max_abs_position_error_rad: `0.012801`
- mean_rms_position_error_rad: `0.004115`
- final_max_abs_position_error_rad: `0.006549`
- controller_state_duration_s: `201.379`
- controller_state_joints: `joint_1, joint_2, joint_3, joint_4, joint_5, joint_6`
- controller_state_max_abs_position_error_rad: `0.018777`
- controller_state_mean_max_abs_position_error_rad: `0.006126`
- controller_state_p95_max_abs_position_error_rad: `0.011498`
- controller_state_mean_rms_position_error_rad: `0.003364`
- controller_state_final_max_abs_position_error_rad: `0.003585`

This observer is passive. It does not publish commands or alter controller behavior.
