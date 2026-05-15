# Research Baseline v0.4 Contact Validation

This note is retained for compatibility with the earlier v0.4 contact
validation draft. The implemented v0.4 validation is the passive contact-probe
trial documented in:

```bash
docs/research_baseline_v0_4_contact_probe_validation.md
```

Use:

```bash
ros2 launch thesis_bringup run_contact_probe_validation_trial.launch.py
```

This validation does not launch the task trajectory executor and does not force
the KUKA robot to touch the peg, hole, target, or validation pad.
