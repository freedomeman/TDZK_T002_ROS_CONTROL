#!/usr/bin/env python3
"""
头部并联机构 IK/FK 解算节点。

FK 调试: /joint_states → FK → /head_solver/fk_pose (0.5s节流)
IK 控制: /target_pose → IK → /t002_controller/command
"""

import os
import sys
from contextlib import contextmanager

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

@contextmanager
def suppress_stdout():
    fd = os.open(os.devnull, os.O_WRONLY)
    old = os.dup(1)
    os.dup2(fd, 1)
    os.close(fd)
    try:
        yield
    finally:
        os.dup2(old, 1)
        os.close(old)

with suppress_stdout():
    from head_solver.head_solver import create_model, compute_state_from_motors, compute_state_from_pose


class SolverNode(Node):
    def __init__(self):
        super().__init__('head_solver')
        self.model = create_model()
        self.fk_counter = 0

        # FK 正解：读电机角 → 算姿态（BEST_EFFORT 避免 buffer 溢出）
        self.sub_joints = self.create_subscription(
            JointState,
            '/joint_states',
            self.fk_callback,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST))

        self.pub_fk = self.create_publisher(Float64MultiArray, '/head_solver/fk_pose', 10)

        # IK 逆解：目标姿态 → 电机目标角
        self.sub_target = self.create_subscription(
            Float64MultiArray, '/target_pose', self.ik_callback, 10)

        self.pub_cmd = self.create_publisher(Float64MultiArray, '/t002_controller/command', 10)

        self.get_logger().info('Head solver ready.')
        self.get_logger().info('  FK: /joint_states → /head_solver/fk_pose')
        self.get_logger().info('  IK: /target_pose  → /t002_controller/command')

    # ── FK 正解 ──────────────────────────────────────────────
    def fk_callback(self, msg: JointState):
        self.fk_counter += 1
        if self.fk_counter % 25 != 0:
            return

        yaw_pos = 0.0
        theta1 = 0.0
        theta2 = 0.0
        for name, pos in zip(msg.name, msg.position):
            if name == 'yaw':   yaw_pos = pos
            elif name == 'pitch': theta1 = pos
            elif name == 'roll':  theta2 = pos

        with suppress_stdout():
            result = compute_state_from_motors(self.model, theta1, theta2, yaw=yaw_pos)

        if result['error_state'] != 0:
            self.get_logger().warn(
                f'FK invalid (error_state={result["error_state"]}) '
                f'motors[y={yaw_pos:.4f} t1={theta1:.4f} t2={theta2:.4f}]')
            return

        fk = Float64MultiArray()
        fk.data = [result['yaw'], result['pitch'], result['roll']]
        self.pub_fk.publish(fk)
        self.get_logger().info(
            f'FK: motors[y={yaw_pos:.4f} t1={theta1:.4f} t2={theta2:.4f}] '
            f'→ pose[y={result["yaw"]:.4f} p={result["pitch"]:.4f} r={result["roll"]:.4f}]')

    # ── IK 逆解 ──────────────────────────────────────────────
    def ik_callback(self, msg: Float64MultiArray):
        if len(msg.data) < 3:
            self.get_logger().warn(f'Need 3 values [yaw,pitch,roll], got {len(msg.data)}')
            return

        yaw, pitch, roll = msg.data[0], msg.data[1], msg.data[2]

        with suppress_stdout():
            result = compute_state_from_pose(self.model, pitch, roll, yaw)

        if result['error_state'] != 0:
            self.get_logger().warn(
                f'IK failed (error_state={result["error_state"]}) '
                f'for pitch={pitch:.3f} roll={roll:.3f} yaw={yaw:.3f}')
            return

        cmd = Float64MultiArray()
        cmd.data = [yaw, result['theta1'], result['theta2']]
        self.pub_cmd.publish(cmd)
        self.get_logger().info(
            f'IK: pose[y={yaw:.4f} p={pitch:.4f} r={roll:.4f}] '
            f'→ motors[y={yaw:.4f} t1={result["theta1"]:.4f} t2={result["theta2"]:.4f}]')


def main(args=None):
    rclpy.init(args=args)
    node = SolverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
