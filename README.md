# Visuomotor Context-Based Meta-RL with Virtual-Force Safety for Peg-in-Hole Assembly

This repository contains the technical development of a doctoral research project on safe and adaptable robotic peg-in-hole assembly for smart manufacturing.

The project focuses on building a proposal-aligned simulation foundation for:

- KUKA LBR iisy 6 R1300 robotic assembly
- Peg-in-hole contact-rich manipulation
- RGB-D perception using a D405-like camera model
- Force/contact sensing and contact-state monitoring
- Safety filtering and virtual-force/admittance interfaces
- ROS 2 Jazzy and Gazebo-based simulation workflows
- Future context-based meta-reinforcement learning experiments

---

## Current Development Focus

The current implementation is focused on building the **proposal simulation cell** before starting learning, policy training, or research result generation.

The current priority is:

```text
Simulation cell → RGB-D sensing → Contact physics → Safety interface → Control → Learning
