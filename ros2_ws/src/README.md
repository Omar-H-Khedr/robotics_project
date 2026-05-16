# Visuomotor Context-Based Meta-RL with Virtual-Force Safety for Peg-in-Hole Assembly

This repository documents the technical development of my doctoral research project on safe and adaptable robotic peg-in-hole assembly for smart manufacturing.

## Project Status

The current implementation focuses on a ROS 2 Jazzy and Gazebo-based research framework for:

- KUKA LBR iisy simulation workcell
- Peg-in-hole task environment
- Joint-space task execution baseline
- Safety monitoring layer
- Experiment logging and trial summaries
- Gazebo contact sensing and contact-force extraction
- Robot-generated contact validation with force guard logic

## Current Stable Milestones

| Version | Description | Status |
|---|---|---|
| v0.1 | Stable Gazebo KUKA workcell baseline | Completed |
| v0.2 | Full task sequence with logging and safety monitor | Completed |
| v0.3 | Contact metrics infrastructure and diagnostics | Completed |
| v0.4 | Minimal Gazebo contact validation world | Completed |
| v0.5 | Contact force extraction from Gazebo Contacts messages | Completed |
| v0.6 | Robot-generated contact validation | Completed |
| v0.7 | Force-threshold diagnostics | Completed |
| v0.8/v0.9 | Force-guarded and early-contact guard experiments | In progress |

## Recommended Launch Commands

### Full research baseline trial

```bash
cd ~/code/robotics_project/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch thesis_bringup run_full_research_trial.launch.py
