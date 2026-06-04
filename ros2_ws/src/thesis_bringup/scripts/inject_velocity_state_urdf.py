#!/usr/bin/env python3
"""Inject a velocity state interface into every joint of a URDF on stdin.

Usage: ``xacro ... | inject_velocity_state_urdf.py > injected.urdf``

This is a diagnostic helper used by the
``inject_velocity_state:=true`` research-baseline diagnostic. The
upstream ``kuka_lbr_iisy_ros2_control_macro`` only declares
``position`` as a state interface, so the JTC's
``position_derivative_gain`` has to fall back to finite-difference of
position. Adding a ``velocity`` state interface to every joint lets
the JTC use real joint velocity from ``gz_ros2_control/GazeboSimSystem``.

Idempotent: re-running on a URDF that already has ``velocity`` is a
no-op.
"""

from __future__ import annotations

import sys
import xml.etree.ElementTree as ET


def inject(urdf_xml: str) -> str:
    """Add a velocity state interface to every joint that lacks one."""
    if "<ros2_control" not in urdf_xml:
        return urdf_xml
    root = ET.fromstring(urdf_xml)
    changed = 0
    for ros2ctrl in root.iter():
        if not str(ros2ctrl.tag).endswith("ros2_control"):
            continue
        for joint in ros2ctrl.findall("joint"):
            existing = {
                iface.get("name")
                for iface in joint.findall("state_interface")
                if iface.get("name") is not None
            }
            if "velocity" in existing:
                continue
            ET.SubElement(joint, "state_interface").set("name", "velocity")
            changed += 1
    if changed:
        print(
            f"injected velocity state_interface into {changed} joint(s)",
            file=sys.stderr,
        )
    return '<?xml version="1.0"?>\n' + ET.tostring(root, encoding="unicode")


def main() -> int:
    urdf_xml = sys.stdin.read()
    sys.stdout.write(inject(urdf_xml))
    return 0


if __name__ == "__main__":
    sys.exit(main())
