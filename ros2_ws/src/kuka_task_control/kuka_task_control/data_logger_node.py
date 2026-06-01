#!/usr/bin/env python3
import os
from datetime import datetime

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Wrench
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class DataLoggerNode(Node):

    _NUM_JOINTS = 6

    def __init__(self) -> None:
        super().__init__('data_logger_node')

        self.declare_parameter('log_rate', 50.0)
        self.declare_parameter('log_dir', '/tmp/thesis_logs')

        self._log_rate: float = self.get_parameter('log_rate').value
        self._log_dir: str = self.get_parameter('log_dir').value

        os.makedirs(self._log_dir, exist_ok=True)

        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._csv_path = os.path.join(
            self._log_dir, f'insertion_log_{timestamp_str}.csv'
        )
        self._csv_file = open(self._csv_path, 'w')
        self._write_header()
        self._rows_written: int = 0

        self._joint_positions: list[float] = [0.0] * self._NUM_JOINTS
        self._joint_velocities: list[float] = [0.0] * self._NUM_JOINTS
        self._wrench_force: list[float] = [0.0, 0.0, 0.0]
        self._wrench_torque: list[float] = [0.0, 0.0, 0.0]
        self._state: str = ''

        self.create_subscription(
            JointState, '/joint_states', self._joint_states_cb, 10,
        )
        self.create_subscription(
            Wrench, '/ft_sensor_wrench', self._wrench_cb, 10,
        )
        self.create_subscription(
            String, '/insertion_state', self._state_cb, 10,
        )

        period = 1.0 / self._log_rate
        self._timer = self.create_timer(period, self._log_tick)

        self.get_logger().info(
            f'DataLoggerNode started.  Writing to {self._csv_path}  '
            f'(rate={self._log_rate:.1f} Hz)'
        )

    def _write_header(self) -> None:
        header = (
            'timestamp,'
            'j1_pos,j2_pos,j3_pos,j4_pos,j5_pos,j6_pos,'
            'j1_vel,j2_vel,j3_vel,j4_vel,j5_vel,j6_vel,'
            'fx,fy,fz,tx,ty,tz,'
            'state'
        )
        self._csv_file.write(header + '\n')

    def _joint_states_cb(self, msg: JointState) -> None:
        if len(msg.position) >= self._NUM_JOINTS:
            self._joint_positions = list(msg.position[:self._NUM_JOINTS])
        if len(msg.velocity) >= self._NUM_JOINTS:
            self._joint_velocities = list(msg.velocity[:self._NUM_JOINTS])

    def _wrench_cb(self, msg: Wrench) -> None:
        self._wrench_force = [msg.force.x, msg.force.y, msg.force.z]
        self._wrench_torque = [msg.torque.x, msg.torque.y, msg.torque.z]

    def _state_cb(self, msg: String) -> None:
        self._state = msg.data

    def _log_tick(self) -> None:
        timestamp = self.get_clock().now().nanoseconds / 1e9

        fields: list[str] = [f'{timestamp:.6f}']
        fields.extend(f'{p:.6f}' for p in self._joint_positions)
        fields.extend(f'{v:.6f}' for v in self._joint_velocities)
        fields.extend(f'{f:.6f}' for f in self._wrench_force)
        fields.extend(f'{t:.6f}' for t in self._wrench_torque)
        fields.append(self._state)

        self._csv_file.write(','.join(fields) + '\n')
        self._rows_written += 1

        if self._rows_written % 100 == 0:
            self._csv_file.flush()

    def destroy_node(self) -> None:
        if self._csv_file and not self._csv_file.closed:
            self._csv_file.flush()
            self._csv_file.close()
            self.get_logger().info(
                f'CSV closed: {self._csv_path}  '
                f'({self._rows_written} rows written)'
            )
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DataLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt \u2013 shutting down.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
