# proposal_simulation_cell_v1_0

Purpose: proposal-aligned simulation-cell foundation for visuomotor context-based meta-RL with virtual-force safety.

Simulation engine: `gazebo`
Isaac Sim available: `False`
Gazebo fallback used: `True`
Robot model: `KUKA LBR iisy 6 R1300`

Safety constraints: motion execution disabled, real robot unused, MoveIt unused, and /compute_ik not called.

Configured assets: table/work surface, peg, hole/block, D405-equivalent RGB-D camera, contact wrench interface, joint states, hole_center, peg_tip, and insertion_axis_z.
