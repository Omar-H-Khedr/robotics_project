"""Shared robot_description helpers for no-motion MoveIt diagnostics."""

from __future__ import annotations

import subprocess
from pathlib import Path
from typing import Any
from xml.etree import ElementTree

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)


RESEARCH_ROBOT_XACRO = "lbr_iisy3_r760_research_gripper.urdf.xacro"
REQUIRED_JOINTS = ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6")


def research_robot_xacro_path() -> Path:
    """Return the same robot xacro used by the full Cartesian diagnostics."""
    try:
        return (
            Path(get_package_share_directory("peg_in_hole_description"))
            / "urdf"
            / RESEARCH_ROBOT_XACRO
        )
    except PackageNotFoundError:
        return (
            Path(__file__).resolve().parents[2]
            / "peg_in_hole_description"
            / "urdf"
            / RESEARCH_ROBOT_XACRO
        )


def robot_description_file_fallback() -> tuple[str, str | None]:
    """Build robot_description from the project xacro without starting motion."""
    xacro_path = research_robot_xacro_path()
    if not xacro_path.is_file():
        return "", f"robot_description xacro not found: {xacro_path}"
    try:
        result = subprocess.run(
            [
                "xacro",
                str(xacro_path),
                "mode:=gazebo",
                "prefix:=",
                "x:=0.80",
                "y:=-0.75",
                "z:=0.75",
                "roll:=0",
                "pitch:=0",
                "yaw:=1.5708",
            ],
            check=True,
            capture_output=True,
            text=True,
            timeout=10.0,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        return "", f"robot_description file fallback failed: {exc}"
    return result.stdout, None


def robot_description_content_report(
    robot_description: str,
    required_joints: tuple[str, ...] = REQUIRED_JOINTS,
) -> dict[str, Any]:
    joint_names: list[str] = []
    link_names: list[str] = []
    parse_error: str | None = None
    if robot_description:
        try:
            root = ElementTree.fromstring(robot_description)
        except ElementTree.ParseError as exc:
            root = None
            parse_error = str(exc)
        if root is not None:
            joint_names = [
                joint.attrib["name"]
                for joint in root.findall("joint")
                if joint.attrib.get("name")
            ]
            link_names = [
                link.attrib["name"]
                for link in root.findall("link")
                if link.attrib.get("name")
            ]

    joint_set = set(joint_names)
    return {
        "robot_description_available": bool(robot_description),
        "robot_description_length": len(robot_description),
        "robot_description_contains_required_joints": all(
            joint in joint_set for joint in required_joints
        ),
        "robot_description_contains_tool0": "tool0" in set(link_names),
        "robot_description_parse_error": parse_error,
    }
