#!/usr/bin/env python3
"""Convert robot URDF to SDF and spawn in Gazebo with gz_ros2_control plugin.

Gazebo Sim (Harmonic) does NOT process <gazebo> URDF extension elements.
This node:
  1. Expands xacro to URDF
  2. Converts URDF → SDF via ``gz sdf -p``
  3. Injects the ``gz_ros2_control-system`` plugin into the SDF
  4. Spawns the SDF using ``ros_gz_sim create -file``
"""

import argparse
import os
import subprocess
import sys
import uuid
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory


_RESOURCE_PACKAGE = "kuka_resources"
_CONTROLLER_CONFIG = "config/fake_hardware_config_6_axis.yaml"


def _resolve_path(spec: str) -> str:
    """Resolve ``$(find <pkg>)/rest`` patterns to absolute paths."""
    if spec.startswith("$(find "):
        end = spec.index(")")
        pkg = spec[7:end]
        rest = spec[end + 1 :].lstrip("/")
        return os.path.join(get_package_share_directory(pkg), rest)
    return spec


def _inject_ft_sensor(root: ET.Element) -> None:
    """Add force_torque <sensor> inside joint ft_sensor_joint.

    With a revolute joint (limits [0,0]), gz sdf -p keeps ft_sensor_joint
    as a proper <joint>, so the sensor can be attached at joint level.
    The joint-level FT sensor gives the net wrench across the flange-gripper
    interface (gripper + peg + contact forces).
    """
    model = root.find("model")
    if model is None:
        return
    for joint in model.findall("joint"):
        if joint.get("name") == "ft_sensor_joint":
            sensor = ET.SubElement(joint, "sensor")
            sensor.set("name", "ft_sensor")
            sensor.set("type", "force_torque")
            e = ET.SubElement(sensor, "always_on")
            e.text = "true"
            e = ET.SubElement(sensor, "update_rate")
            e.text = "100"
            e = ET.SubElement(sensor, "visualize")
            e.text = "false"
            ft = ET.SubElement(sensor, "force_torque")
            e = ET.SubElement(ft, "frame")
            e.text = "child"
            e = ET.SubElement(ft, "measure_direction")
            e.text = "child_to_parent"
            break


def _inject_ros2_control_plugin(
    root: ET.Element,
    controller_config: str,
    position_proportional_gain: float,
) -> None:
    """Add gz_ros2_control-system <plugin> inside <model>."""
    model = root.find("model")
    if model is None:
        raise RuntimeError("No <model> element found in converted SDF")

    plugin = ET.SubElement(model, "plugin")
    plugin.set("filename", "gz_ros2_control-system")
    plugin.set("name", "gz_ros2_control::GazeboSimROS2ControlPlugin")

    e = ET.SubElement(plugin, "robot_param")
    e.text = "robot_description"

    e = ET.SubElement(plugin, "robot_param_node")
    e.text = "robot_state_publisher"

    e = ET.SubElement(plugin, "parameters")
    e.text = controller_config

    e = ET.SubElement(plugin, "position_proportional_gain")
    e.text = str(position_proportional_gain)


def _extract_initial_positions(urdf_xml: str) -> dict[str, float]:
    """Extract joint initial positions from URDF <ros2_control> section.

    Returns a dict mapping joint_name -> initial_value for all joints that
    have a ``<state_interface name="position">`` with a numeric
    ``<param name="initial_value">``.
    """
    root = ET.fromstring(urdf_xml)
    result: dict[str, float] = {}
    for joint in root.findall(".//ros2_control/joint"):
        name = joint.get("name")
        if not name:
            continue
        for state_iface in joint.findall("state_interface"):
            if state_iface.get("name") != "position":
                continue
            param = state_iface.find("param[@name='initial_value']")
            if param is not None and param.text:
                try:
                    val = float(param.text.strip())
                except ValueError:
                    continue
                result[name] = val
    return result


def _inject_initial_positions(
    model: ET.Element, initial_positions: dict[str, float]
) -> None:
    """Add ``<initial_position>`` to each joint that has one in the map."""
    for joint in model.findall("joint"):
        name = joint.get("name")
        if name is None or name not in initial_positions:
            continue
        axis = joint.find("axis")
        if axis is None:
            continue
        # gz sdf -p may have already placed an <initial_position>; update it.
        existing = axis.find("initial_position")
        if existing is not None:
            existing.text = str(initial_positions[name])
        else:
            e = ET.SubElement(axis, "initial_position")
            e.text = str(initial_positions[name])


def _inject_plugin(
    sdf_xml: str,
    urdf_xml: str,
    controller_config: str,
    position_proportional_gain: float,
) -> str:
    """Add ros2_control plugin, FT sensor, and initial positions to the SDF."""
    root = ET.fromstring(sdf_xml)
    initial_positions = _extract_initial_positions(urdf_xml)
    model = root.find("model")
    if model is not None:
        _inject_initial_positions(model, initial_positions)
    _inject_ros2_control_plugin(root, controller_config, position_proportional_gain)
    _inject_ft_sensor(root)
    return '<?xml version="1.0"?>\n' + ET.tostring(root, encoding="unicode")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Spawn robot from xacro as SDF with gz_ros2_control plugin"
    )
    parser.add_argument("--xacro", required=True, help="Path to xacro file")
    parser.add_argument("--name", default="lbr_iisy6_r1300", help="Model name")
    parser.add_argument(
        "--xacro-args",
        nargs="*",
        default=[],
        help="Extra args for xacro (e.g. mode:=gazebo)",
    )
    parser.add_argument("--x", default="0.0")
    parser.add_argument("--y", default="0.0")
    parser.add_argument("--z", default="0.0")
    parser.add_argument("--R", default="0.0")
    parser.add_argument("--P", default="0.0")
    parser.add_argument("--Y", default="0.0")
    parser.add_argument("--allow-renaming", action="store_true")
    parser.add_argument(
        "--position-gain",
        type=float,
        default=1000.0,
        help="gz_ros2_control position_proportional_gain",
    )
    parser.add_argument(
        "--controller-config-package",
        default=_RESOURCE_PACKAGE,
        help="Package containing the gz_ros2_control controller YAML.",
    )
    parser.add_argument(
        "--controller-config-path",
        default=_CONTROLLER_CONFIG,
        help="Path to the gz_ros2_control controller YAML inside the package share.",
    )
    args, _ = parser.parse_known_args()

    # 1. Expand xacro → URDF
    xacro_cmd = ["xacro", args.xacro] + args.xacro_args
    urdf = subprocess.run(xacro_cmd, capture_output=True, text=True)
    if urdf.returncode != 0:
        print(f"xacro failed:\n{urdf.stderr}", file=sys.stderr)
        sys.exit(1)

    # 2. Write URDF to a temp file for gz sdf -p
    urdf_path = f"/tmp/_spawn_urdf_{uuid.uuid4().hex}.urdf"
    with open(urdf_path, "w") as f:
        f.write(urdf.stdout)

    # 3. Convert URDF → SDF via gz sdf -p
    sdf_raw = subprocess.run(
        ["gz", "sdf", "-p", urdf_path], capture_output=True, text=True
    )
    os.unlink(urdf_path)

    if sdf_raw.returncode != 0:
        # gz sdf -p may exit non-zero for SDF validity warnings (e.g. camera
        # frame graph disconnected from model) while still producing valid SDF
        # XML on stdout.  Only hard-fail if stdout is empty.
        if not sdf_raw.stdout.strip() or "<sdf" not in sdf_raw.stdout:
            print(f"gz sdf -p failed:\n{sdf_raw.stderr}", file=sys.stderr)
            sys.exit(1)
        print(f"gz sdf -p warnings (ignored):\n{sdf_raw.stderr}", file=sys.stderr)

    # 4. Inject plugin & initial positions into SDF
    controller_config_path = os.path.join(
        get_package_share_directory(args.controller_config_package),
        args.controller_config_path,
    )
    sdf_with_plugin = _inject_plugin(
        sdf_raw.stdout,
        urdf.stdout,
        controller_config_path,
        args.position_gain,
    )

    # 5. Spawn via ros_gz_sim create (use -string to avoid temp-file races)
    create_args = [
        "ros2",
        "run",
        "ros_gz_sim",
        "create",
        "-string",
        sdf_with_plugin,
        "-name",
        args.name,
        "-x",
        args.x,
        "-y",
        args.y,
        "-z",
        args.z,
        "-R",
        args.R,
        "-P",
        args.P,
        "-Y",
        args.Y,
    ]
    if args.allow_renaming:
        create_args.extend(["-allow_renaming"])

    spawned = subprocess.run(create_args, capture_output=True, text=True)

    if spawned.stdout:
        print(spawned.stdout)
    if spawned.stderr:
        print(spawned.stderr, file=sys.stderr)

    if spawned.returncode != 0:
        sys.exit(spawned.returncode)


if __name__ == "__main__":
    main()
