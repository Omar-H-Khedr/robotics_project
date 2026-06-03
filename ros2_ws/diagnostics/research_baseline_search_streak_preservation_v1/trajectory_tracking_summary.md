# Trajectory Tracking Summary

- state_topic: `/joint_trajectory_controller/controller_state`
- command_topic: `/joint_trajectory_controller/joint_trajectory`
- joint_state_topic: `/joint_states`
- observed_commands: `10`
- jtc_state_samples: `50125`
- jtc_state_records: `50125`
- samples: `49671`
- duration_s: `198.680`
- joints: `joint_1, joint_2, joint_3, joint_4, joint_5, joint_6`
- max_abs_position_error_rad: `0.133247`
- mean_max_abs_position_error_rad: `0.007132`
- p95_max_abs_position_error_rad: `0.012255`
- mean_rms_position_error_rad: `0.003795`
- final_max_abs_position_error_rad: `0.003332`
- controller_state_duration_s: `200.616`
- controller_state_joints: `joint_1, joint_2, joint_3, joint_4, joint_5, joint_6`
- controller_state_max_abs_position_error_rad: `0.018926`
- controller_state_mean_max_abs_position_error_rad: `0.006089`
- controller_state_p95_max_abs_position_error_rad: `0.011391`
- controller_state_mean_rms_position_error_rad: `0.003344`
- controller_state_final_max_abs_position_error_rad: `0.003332`

This observer is passive. It does not publish commands or alter controller behavior.
