#!/usr/bin/env python3
"""Generate parameterized peg-in-hole Gazebo SDF world files.

Creates world SDF files with configurable peg diameter, hole diameter,
and clearance. The hole collision geometry uses 4 box segments whose
inner edges approximate the circular opening.

Usage:
    python3 generate_scenario_world.py \
        --peg-radius 0.0125 --hole-radius 0.0135 \
        --output /tmp/scenario_world.sdf

    python3 generate_scenario_world.py --scenario-config scenario.yaml
"""

import argparse
import os
import xml.etree.ElementTree as ET
from typing import Optional

# Canonical world template (simplified, no includes — we inline the models)
WORLD_TEMPLATE = """\
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="peg_in_hole_world">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <plugin name="gz::sim::systems::Physics" filename="ignition-gazebo-physics-system"/>
    <plugin name="gz::sim::systems::UserCommands" filename="ignition-gazebo-user-commands-system"/>
    <plugin name="gz::sim::systems::SceneBroadcaster" filename="ignition-gazebo-scene-broadcaster-system"/>
    <plugin name="gz::sim::systems::Contact" filename="ignition-gazebo-contact-system"/>
    <plugin name="gz::sim::systems::Sensors" filename="ignition-gazebo-sensors-system">
      <render_engine>ogre</render_engine>
    </plugin>
    <plugin name="gz::sim::systems::ForceTorque" filename="gz-sim8-forcetorque-system"/>

    <gravity>0 0 -9.81</gravity>
    <magnetic_field>6.0e-6 2.3e-5 -4.2e-5</magnetic_field>
    <atmosphere type="adiabatic"/>

    <scene>
      <ambient>0.45 0.45 0.45 1</ambient>
      <background>0.72 0.74 0.76 1</background>
      <shadows>true</shadows>
    </scene>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material>
            <ambient>0.55 0.57 0.58 1</ambient>
            <diffuse>0.62 0.64 0.65 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <include>
      <name>work_table</name>
      <uri>model://work_table</uri>
      <pose>0.80 0 0 0 0 0</pose>
    </include>

    <include>
      <name>robot_pedestal</name>
      <uri>model://robot_pedestal</uri>
      <pose>0.80 -0.75 0.0 0 0 0</pose>
    </include>

    {hole_fixture}

    {target_plate}

    <light name="key_area_light" type="point">
      <pose>0.20 -0.60 1.80 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1.4</intensity>
      <diffuse>0.95 0.95 0.90 1</diffuse>
      <specular>0.35 0.35 0.35 1</specular>
      <attenuation>
        <range>5.0</range>
        <constant>0.6</constant>
        <linear>0.15</linear>
        <quadratic>0.02</quadratic>
      </attenuation>
    </light>

    <model name="d405_rgbd_camera">
      <static>true</static>
      <pose>0.42 -0.55 1.18 0.95 0 0.35</pose>
      <link name="camera_link">
        <visual name="camera_visual">
          <geometry><box><size>0.03 0.025 0.02</size></box></geometry>
          <material><ambient>0.03 0.03 0.03 1</ambient><diffuse>0.05 0.05 0.05 1</diffuse></material>
        </visual>
        <sensor name="d405_color" type="camera">
          <always_on>true</always_on>
          <update_rate>30</update_rate>
          <topic>/d405/color/image_raw</topic>
          <camera>
            <horizontal_fov>1.22</horizontal_fov>
            <image><width>848</width><height>480</height><format>R8G8B8</format></image>
            <clip><near>0.07</near><far>2.0</far></clip>
          </camera>
        </sensor>
        <sensor name="d405_depth" type="depth_camera">
          <always_on>true</always_on>
          <update_rate>30</update_rate>
          <topic>/d405/depth/image_rect_raw</topic>
          <camera>
            <horizontal_fov>1.22</horizontal_fov>
            <image><width>848</width><height>480</height><format>R_FLOAT32</format></image>
            <clip><near>0.07</near><far>2.0</far></clip>
          </camera>
        </sensor>
      </link>
    </model>

    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>0.8</intensity>
      <direction>-0.4 0.2 -0.9</direction>
      <diffuse>0.80 0.82 0.85 1</diffuse>
      <specular>0.20 0.20 0.20 1</specular>
    </light>
  </world>
</sdf>
"""


def _generate_hole_fixture_sdf(hole_radius: float) -> str:
    """Generate hole_fixture SDF with given hole radius.

    The fixture is 180mm square, 40mm tall, sitting on the 0.75m table.
    The 4-box collision segments approximate a circular hole opening.
    Inner edges of the boxes define the effective hole diameter.
    """
    # Box half-size for the 4 segments (same proportions as original)
    box_half_x = 0.036  # half-width of left/right boxes
    box_half_y = 0.036  # half-depth of front/rear boxes

    # Box center offsets: inner edge at hole_radius from center
    front_y = hole_radius + box_half_y
    rear_y = -(hole_radius + box_half_y)
    left_x = -(hole_radius + box_half_x)
    right_x = hole_radius + box_half_x

    # Front/rear boxes span the full plate width
    front_rear_w = 0.18
    # Left/right boxes have same width as front/rear (box_half_x * 2)
    left_right_w = 2 * box_half_x

    return f"""\
    <model name="hole_fixture">
      <static>true</static>
      <link name="fixture_link">
        <pose>0 0 0.02 0 0 0</pose>
        <collision name="hole_fixture_collision">
          <pose>0 {front_y:.6f} 0 0 0 0</pose>
          <geometry><box><size>{front_rear_w:.6f} {2*box_half_y:.6f} 0.04</size></box></geometry>
        </collision>
        <collision name="fixture_rear_collision">
          <pose>0 {rear_y:.6f} 0 0 0 0</pose>
          <geometry><box><size>{front_rear_w:.6f} {2*box_half_y:.6f} 0.04</size></box></geometry>
        </collision>
        <collision name="fixture_left_collision">
          <pose>{left_x:.6f} 0 0 0 0 0</pose>
          <geometry><box><size>{left_right_w:.6f} {2*box_half_y:.6f} 0.04</size></box></geometry>
        </collision>
        <collision name="fixture_right_collision">
          <pose>{right_x:.6f} 0 0 0 0 0</pose>
          <geometry><box><size>{left_right_w:.6f} {2*box_half_y:.6f} 0.04</size></box></geometry>
        </collision>
        <sensor name="hole_contact_sensor" type="contact">
          <always_on>true</always_on>
          <update_rate>100.0</update_rate>
          <topic>/gazebo/contacts/hole</topic>
          <contact>
            <collision>hole_fixture_collision</collision>
            <collision>fixture_rear_collision</collision>
            <collision>fixture_left_collision</collision>
            <collision>fixture_right_collision</collision>
          </contact>
        </sensor>
        <visual name="fixture_visual">
          <geometry><box><size>0.18 0.18 0.04</size></box></geometry>
          <material>
            <ambient>0.18 0.22 0.24 1</ambient>
            <diffuse>0.24 0.30 0.32 1</diffuse>
            <specular>0.20 0.22 0.22 1</specular>
          </material>
        </visual>
        <visual name="fixture_socket_visual">
          <pose>0 0 0.021 0 0 0</pose>
          <geometry><cylinder><radius>{hole_radius:.6f}</radius><length>0.004</length></cylinder></geometry>
          <material>
            <ambient>0.02 0.02 0.02 1</ambient>
            <diffuse>0.03 0.03 0.03 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def _generate_target_plate_sdf(hole_radius: float) -> str:
    """Generate target_plate SDF with given hole radius.

    The plate is 180mm square, 20mm thick, sitting on the fixture at z=0.79m.
    The 4-box collision segments approximate the circular hole opening.
    """
    box_half_x = 0.036
    box_half_y = 0.036

    front_y = hole_radius + box_half_y
    rear_y = -(hole_radius + box_half_y)
    left_x = -(hole_radius + box_half_x)
    right_x = hole_radius + box_half_x

    front_rear_w = 0.18
    left_right_w = 2 * box_half_x

    return f"""\
    <model name="target_plate">
      <static>true</static>
      <link name="plate_link">
        <pose>0 0 0.01 0 0 0</pose>
        <collision name="target_plate_collision">
          <pose>0 {front_y:.6f} 0 0 0 0</pose>
          <geometry><box><size>{front_rear_w:.6f} {2*box_half_y:.6f} 0.02</size></box></geometry>
        </collision>
        <collision name="plate_rear_collision">
          <pose>0 {rear_y:.6f} 0 0 0 0</pose>
          <geometry><box><size>{front_rear_w:.6f} {2*box_half_y:.6f} 0.02</size></box></geometry>
        </collision>
        <collision name="plate_left_collision">
          <pose>{left_x:.6f} 0 0 0 0 0</pose>
          <geometry><box><size>{left_right_w:.6f} {2*box_half_y:.6f} 0.02</size></box></geometry>
        </collision>
        <collision name="plate_right_collision">
          <pose>{right_x:.6f} 0 0 0 0 0</pose>
          <geometry><box><size>{left_right_w:.6f} {2*box_half_y:.6f} 0.02</size></box></geometry>
        </collision>
        <sensor name="target_contact_sensor" type="contact">
          <always_on>true</always_on>
          <update_rate>100.0</update_rate>
          <topic>/gazebo/contacts/target</topic>
          <contact>
            <collision>target_plate_collision</collision>
            <collision>plate_rear_collision</collision>
            <collision>plate_left_collision</collision>
            <collision>plate_right_collision</collision>
          </contact>
        </sensor>
        <visual name="plate_visual">
          <geometry><box><size>0.18 0.18 0.02</size></box></geometry>
          <material>
            <ambient>0.18 0.28 0.36 1</ambient>
            <diffuse>0.24 0.40 0.50 1</diffuse>
            <specular>0.25 0.25 0.25 1</specular>
          </material>
        </visual>
        <visual name="hole_opening_visual">
          <pose>0 0 0.0105 0 0 0</pose>
          <geometry><cylinder><radius>{hole_radius:.6f}</radius><length>0.003</length></cylinder></geometry>
          <material>
            <ambient>0.01 0.01 0.01 1</ambient>
            <diffuse>0.02 0.02 0.02 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def generate_world(
    peg_radius: float,
    hole_radius: float,
    peg_length: float = 0.11,
    hole_center_xy: tuple = (0.52, -0.20),
    fixture_z: float = 0.75,
    plate_z: float = 0.79,
    output_path: Optional[str] = None,
) -> str:
    """Generate a complete SDF world file with parameterized geometry.

    Args:
        peg_radius: Peg radius in metres (e.g., 0.0125 for 25mm dia)
        hole_radius: Hole radius in metres (e.g., 0.0135 for 27mm dia)
        peg_length: Peg length in metres (default 0.11)
        hole_center_xy: (x, y) position of hole center in world frame
        fixture_z: Z position of fixture bottom (default 0.75 = table surface)
        plate_z: Z position of plate bottom (default 0.79 = on top of fixture)
        output_path: If set, write SDF to this file

    Returns:
        SDF string
    """
    clearance = hole_radius - peg_radius

    fixture_sdf = _generate_hole_fixture_sdf(hole_radius)
    plate_sdf = _generate_target_plate_sdf(hole_radius)

    sdf = WORLD_TEMPLATE.format(
        hole_fixture=fixture_sdf,
        target_plate=plate_sdf,
    )

    # Update fixture and plate poses
    cx, cy = hole_center_xy
    sdf = sdf.replace(
        '<name>hole_fixture</name>\n      <uri>model://hole_fixture</uri>\n      <pose>0.52 -0.20 0.75 0 0 0</pose>',
        f'<name>hole_fixture</name>\n      <uri>model://hole_fixture</uri>\n      <pose>{cx:.6f} {cy:.6f} {fixture_z:.6f} 0 0 0</pose>',
    )
    sdf = sdf.replace(
        '<name>target_plate</name>\n      <uri>model://target_plate</uri>\n      <pose>0.52 -0.20 0.79 0 0 0</pose>',
        f'<name>target_plate</name>\n      <uri>model://target_plate</uri>\n      <pose>{cx:.6f} {cy:.6f} {plate_z:.6f} 0 0 0</pose>',
    )

    if output_path:
        os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
        with open(output_path, "w", encoding="utf-8") as f:
            f.write(sdf)

    return sdf


def generate_peg_xacro_args(
    peg_radius: float,
    peg_length: float = 0.11,
) -> str:
    """Generate xacro arguments for the gripper with parameterized peg.

    Returns the xacro property overrides as a string for launch argument use.
    """
    return f"peg_radius:={peg_radius} peg_length:={peg_length}"


def main():
    parser = argparse.ArgumentParser(description="Generate parameterized peg-in-hole world SDF")
    parser.add_argument("--peg-radius", type=float, default=0.0125,
                        help="Peg radius in metres (default: 0.0125 = 25mm dia)")
    parser.add_argument("--hole-radius", type=float, default=0.0135,
                        help="Hole radius in metres (default: 0.0135 = 27mm dia)")
    parser.add_argument("--peg-length", type=float, default=0.11,
                        help="Peg length in metres (default: 0.11)")
    parser.add_argument("--hole-center-x", type=float, default=0.52,
                        help="Hole center X in world frame (default: 0.52)")
    parser.add_argument("--hole-center-y", type=float, default=-0.20,
                        help="Hole center Y in world frame (default: -0.20)")
    parser.add_argument("--output", type=str, default=None,
                        help="Output SDF file path")
    parser.add_argument("--print-info", action="store_true",
                        help="Print geometry info without generating file")

    args = parser.parse_args()

    clearance = args.hole_radius - args.peg_radius

    if args.print_info:
        print(f"Peg diameter:     {2*args.peg_radius*1000:.1f} mm")
        print(f"Hole diameter:    {2*args.hole_radius*1000:.1f} mm")
        print(f"Radial clearance: {clearance*1000:.2f} mm")
        print(f"Clearance ratio:  {clearance/args.peg_radius*100:.1f}%")
        return

    output = args.output or f"/tmp/peg_in_hole_world_r{args.peg_radius:.4f}_h{args.hole_radius:.4f}.sdf"

    sdf = generate_world(
        peg_radius=args.peg_radius,
        hole_radius=args.hole_radius,
        peg_length=args.peg_length,
        hole_center_xy=(args.hole_center_x, args.hole_center_y),
        output_path=output,
    )

    print(f"Generated: {output}")
    print(f"Peg diameter:     {2*args.peg_radius*1000:.1f} mm")
    print(f"Hole diameter:    {2*args.hole_radius*1000:.1f} mm")
    print(f"Radial clearance: {clearance*1000:.2f} mm")


if __name__ == "__main__":
    main()
