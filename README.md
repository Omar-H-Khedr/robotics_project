# Visuomotor Context-Based Meta-Reinforcement Learning for Safe Peg-in-Hole Assembly

**Repository status:** active doctoral research prototype  
**Latest documented sprint:** `proposal_simulation_cell_v2_11_multimodal_contact_observation_logging`  
**README last updated:** 2026-05-26  
**Execution scope:** simulation-first validation only; no real-robot claim is made in this repository state.

---

## 1. Project Overview

This repository contains a ROS 2 / Gazebo-based research framework for robotic peg-in-hole assembly in smart manufacturing cells. The project investigates how a robot can adapt to product variation, tolerance uncertainty, and contact-state changes using:

- Multi-modal observation logging from RGB-D, depth, camera info, robot state, and contact events.
- Context-based meta-reinforcement learning concepts for fast online adaptation.
- A virtual-force / admittance-style safety layer for bounded contact execution.
- Runtime safety monitoring and empirical transfer gates.
- Simulation-backed validation before any real-robot transfer.

The target doctoral research direction is:

> **Visuomotor Context-Based Meta-Reinforcement Learning with Virtual-Force Safety for Adaptable Peg-in-Hole Assembly in Smart Manufacturing**

The current implementation focuses on reproducible simulation infrastructure, launch files, observation logging, contact-transition evidence, and validation artifacts.

---

## 2. Latest Validated Sprint

### Sprint Name

```text
proposal_simulation_cell_v2_11_multimodal_contact_observation_logging
```

### Latest Runtime Result

The latest reported run completed successfully with the following outcome:

```json
{
  "status": "multimodal_contact_observation_logging_validated",
  "scenario_count": 5,
  "scenarios_attempted": 5,
  "scenarios_validated": 5,
  "rgb_topic_available": true,
  "depth_topic_available": true,
  "camera_info_available": true,
  "observation_row_count": 65,
  "contact_transition_row_count": 5
}
```

### Validation Summary

| Item | Result |
|---|---:|
| Build result | Success |
| Packages built | 4 |
| Scenarios attempted | 5 |
| Scenarios validated | 5 |
| RGB topic available | Yes |
| Depth topic available | Yes |
| Camera info available | Yes |
| Observation rows logged | 65 |
| Contact transition rows logged | 5 |
| Fake dataset used | No |
| Fake result claimed | No |
| Real robot used | No |
| Physical endpoint used | No |
| Forceful contact used | No |
| Physical peg insertion claimed | No |

### Known Runtime Note

`move_group` may log a shutdown-time segmentation fault after diagnostics are already written. In the latest reported run, this occurred after the validation artifacts had already been generated and did not invalidate the recorded sprint evidence.

---

## 3. Research Motivation

Smart manufacturing cells increasingly need robotic assembly skills that remain reliable under:

- Product changes.
- Hole and peg tolerance variation.
- Pose uncertainty.
- Partial observability.
- Contact-state ambiguity.
- Edge contact, jamming, wedging, and recovery cases.

Pure geometric planning is insufficient once contact begins. Robust assembly requires closed-loop sensing, safe contact handling, and systematic validation under realistic uncertainty.

This repository therefore focuses on the software and simulation backbone needed to test adaptive, safety-aware assembly policies before transferring them to a physical robot.

---

## 4. Target Robot and Sensors

The intended physical platform for later-stage transfer is:

- **Robot:** KUKA LBR iisy 6 R1300
- **External RGB-D sensor:** Intel RealSense D405
- **Software middleware:** ROS 2 Jazzy
- **Simulation:** Gazebo
- **Optional future simulation backend:** NVIDIA Isaac Sim

Current repository status remains simulation-focused. Real-robot deployment is not claimed unless explicitly validated in a future sprint.

---

## 5. Main Technical Components

### 5.1 Multi-Modal Observation Pipeline

The latest sprint validates logging of:

- RGB topic availability.
- Depth topic availability.
- Camera info availability.
- Scenario-level observation records.
- Contact-transition records.
- Validation status summaries.

This supports later training and evaluation of policies that use visual, geometric, proprioceptive, and contact-related state information.

### 5.2 Safety Layer

The project uses a safety-focused execution concept with two levels:

1. **Runtime safety monitoring**
   - Monitors task phase and relevant state signals.
   - Supports pre-contact and contact-aware constraints.

2. **Virtual-force / admittance-style interaction layer**
   - Intended to mediate contact behavior.
   - Avoids strong “guarantee” claims.
   - Uses bounded operational constraints and empirical safety gates.

The current repository state validates logging and simulation infrastructure rather than final real-world safety performance.

### 5.3 Context-Based Meta-RL Direction

The doctoral method is planned around context-based adaptation:

- A policy should infer task context from short interaction history.
- Context may represent hole/peg variation, misalignment, friction, clearance, or contact regime.
- Future ablations may compare deterministic context encoders with PEARL-style variational encoders.
- The current sprint does not yet claim full meta-RL training completion.

### 5.4 Simulation and Digital Twin Direction

The repository is being developed as a simulation-backed experimental cell:

- Gazebo provides reproducible simulation execution.
- ROS 2 launch files coordinate robot, task, perception, and safety packages.
- Scenario validation is logged into machine-readable artifacts.
- Domain randomization and empirical transfer gates are planned for later stages.

---

## 6. Repository Structure

The workspace is expected to follow a ROS 2 structure similar to:

```text
robotics_project/
└── ros2_ws/
    ├── src/
    │   ├── thesis_bringup/
    │   ├── safety_layer/
    │   ├── perception_pipeline/
    │   ├── experiment_manager/
    │   ├── kuka_task_control/
    │   └── peg_in_hole_description/
    ├── build/
    ├── install/
    └── log/
```

Main package roles:

| Package | Role |
|---|---|
| `thesis_bringup` | Main launch orchestration for research scenarios |
| `safety_layer` | Runtime safety monitoring and safety-state logic |
| `perception_pipeline` | Sensor topic handling and observation support |
| `experiment_manager` | Scenario execution, validation, and evidence logging |
| `kuka_task_control` | Task-level control and trajectory configuration |
| `peg_in_hole_description` | Assembly-cell description assets and simulation models |

---

## 7. Build Instructions

From the ROS 2 workspace:

```bash
cd /home/omar/code/robotics_project/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

Expected latest reported build result:

```text
Build: success, 4 packages finished.
```

---

## 8. Run Latest Validated Sprint

Use the following launch command:

```bash
cd /home/omar/code/robotics_project/ros2_ws
source install/setup.bash
ros2 launch thesis_bringup proposal_simulation_cell_v2_11_multimodal_contact_observation_logging.launch.py
```

Expected high-level outcome:

```text
5 scenarios attempted.
5 scenarios validated.
RGB-D was available.
Observation rows: 65.
Contact transition rows: 5.
No fake dataset.
No fake result.
No real robot.
No physical endpoint.
No peg insertion.
No forceful contact.
```

---

## 9. Evidence and Diagnostics

The latest sprint should produce validation evidence similar to:

```text
multimodal_contact_observation_status.json
observation logs / CSV records
contact transition logs / CSV records
ROS 2 launch logs
Gazebo / MoveIt runtime logs
```

The key status artifact is:

```text
multimodal_contact_observation_status.json
```

The most important validation fields are:

- `status`
- `scenario_count`
- `scenarios_attempted`
- `scenarios_validated`
- `rgb_topic_available`
- `depth_topic_available`
- `camera_info_available`
- `observation_row_count`
- `contact_transition_row_count`

---

## 10. What This Repository Currently Claims

This repository currently claims:

- A ROS 2 / Gazebo simulation cell has been built and launched.
- The latest multimodal contact-observation sprint was validated.
- RGB-D topic availability was confirmed.
- Five simulation scenarios were attempted and validated.
- Observation and contact-transition logs were generated.
- The project has a reproducible launch command for the latest sprint.

This repository does **not** currently claim:

- Real-robot execution.
- Physical peg insertion.
- Physical endpoint validation.
- Forceful real-world contact.
- Final trained meta-RL policy performance.
- Safety guarantees beyond specified simulation assumptions and logged empirical checks.

---

## 11. Planned Next Steps

Recommended next development stages:

1. Stabilize the latest simulation launch and archive its validation artifacts.
2. Add structured evidence folders for every sprint.
3. Add scenario configuration files for controlled tolerance and misalignment variation.
4. Add baseline controller comparisons.
5. Add safe-success metrics.
6. Add context encoder experiments.
7. Add domain-randomization protocols.
8. Add simulation-to-real transfer gates.
9. Only then proceed to physical robot validation.

---

## 12. Suggested Evidence Folder Convention

Recommended structure for future sprints:

```text
evidence/
└── proposal_simulation_cell_v2_11_multimodal_contact_observation_logging/
    ├── multimodal_contact_observation_status.json
    ├── observations.csv
    ├── contact_transitions.csv
    ├── launch_command.txt
    ├── build_summary.txt
    └── notes.md
```

Each sprint should include:

- Exact launch command.
- Build status.
- Number of scenarios attempted.
- Number of scenarios validated.
- Important ROS topic availability checks.
- Any known runtime warnings.
- Clear statement of what is and is not claimed.

---

## 13. Recommended Git Workflow

Before pushing a new sprint, update this `README.md` first.

Recommended commit sequence:

```bash
git add README.md
git commit -m "docs: update README for v2.11 multimodal contact observation sprint"

git add .
git status
git commit -m "feat: add v2.11 multimodal contact observation logging evidence"

git push origin main
```

If this is the first push to a new GitHub repository:

```bash
git init
git branch -M main
git remote add origin https://github.com/USERNAME/REPOSITORY_NAME.git

git add README.md
git commit -m "docs: add main README with latest sprint status"

git add .
git commit -m "feat: add ROS 2 simulation workspace"

git push -u origin main
```

---

## 14. Citation and Proposal Alignment Notes

The implementation aligns with the doctoral proposal direction:

- Smart manufacturing robotic assembly.
- Peg-in-hole contact uncertainty.
- Multi-modal sensing.
- Context-based adaptation.
- Safety-constrained execution.
- Simulation-backed validation.

The wording intentionally avoids overclaiming. Use terms such as:

- “safety constraints under specified assumptions”
- “empirical safety gates”
- “bounded operational constraints”
- “simulation-backed validation”
- “validated simulation evidence”

Avoid unsupported claims such as:

- “guaranteed safe”
- “fully solved peg-in-hole”
- “real-robot validated”
- “physical endpoint proven”

unless those claims are supported by future evidence.

---

## 15. Maintainer Notes

Keep the README synchronized with the latest validated sprint. Every future push should update:

- Latest sprint name.
- Build status.
- Launch command.
- Validation artifact names.
- Scenario counts.
- Topic availability.
- Known runtime notes.
- Clear claims and non-claims.

