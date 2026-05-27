# proposal_simulation_cell_v2_15_context_action_ablation_validation

Status: `context_action_ablation_validated`

This diagnostic validates a Gazebo-only batch of no-contact standby, bounded approach-to-contact, stop-on-contact, retreat, and return-to-ready scenarios.

- source_context_sprint: v2.13
- source_action_sprint: v2.14
- context_embedding_count: 5
- scenario_count: 5
- expected_run_count: 10
- runs_attempted: 10
- runs_validated: 10
- baseline_runs_validated: 5
- context_conditioned_runs_validated: 5
- paired_comparison_written: true
- all_action_parameters_within_bounds: true
- all_initial_no_contact_verified: true
- all_contact_triggers_after_motion: true
- all_stop_on_contact_executed: true
- all_retreats_completed: true
- all_post_retreat_no_contact_verified: true
- max_observed_force_n: 0.093922079
- failed_scenarios: []
- peg_insertion_executed: false
- forceful_contact_executed: false
- real_robot_used: false
