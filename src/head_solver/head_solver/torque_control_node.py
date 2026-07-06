#!/usr/bin/env python3
"""
头部并联机构 力矩控制节点

通过读取电机角度/速度 → FK 反算实际姿态 → 姿态 PD → Jacobian 反解电机力矩
发布到 /t002_controller/command

用法: ros2 run head_solver torque_control_node
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
    from head_solver.head_solver import (
        create_model,
        compute_motor_torque_command,
        HeadControlGains,
    )


class TorqueControlNode(Node):
    def __init__(self):
        super().__init__('torque_control_node')

        # ── 并联机构模型 ──
        with suppress_stdout():
            self.model = create_model()

        # ── PD 增益 (姿态空间) ──
        self.declare_parameter('kp_pitch', 0.2)
        self.declare_parameter('kd_pitch', 0.0)
        self.declare_parameter('kp_roll', 0.2)
        self.declare_parameter('kd_roll', 0.0)
        self.declare_parameter('max_joint_torque', 2.0)   # pitch/roll 力矩限幅
        self.declare_parameter('max_motor_torque', 1.5)   # 电机力矩限幅 N·m

        # ── 状态缓存 ──
        self.theta1 = 0.0       # pitch 电机角度
        self.theta2 = 0.0       # roll 电机角度
        self.theta1_dot = 0.0   # pitch 电机速度
        self.theta2_dot = 0.0   # roll 电机速度
        self.yaw_pos = 0.0

        # 目标姿态
        self.target_pitch = 0.0
        self.target_roll = 0.0
        self.target_yaw = 0.0
        self.has_target = True  # 默认启用, 零目标

        # 上一帧实际姿态 (FK 初值)
        self.last_pitch = 0.0
        self.last_roll = 0.0

        # ── 订阅 /joint_states ──
        self.sub_joints = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST))

        # ── 订阅 /target_pose ──
        self.sub_target = self.create_subscription(
            Float64MultiArray,
            '/target_pose',
            self.target_callback,
            10)

        # ── 发布力矩命令 ──
        self.pub_cmd = self.create_publisher(
            Float64MultiArray, '/t002_controller/command', 10)

        # ── 调试发布 ──
        self.pub_debug = self.create_publisher(
            Float64MultiArray, '/head_solver/torque_debug', 10)

        self.get_logger().info('Torque control node ready.')
        self.get_logger().info('  Sub: /joint_states, /target_pose')
        self.get_logger().info('  Pub: /t002_controller/command')

    def target_callback(self, msg: Float64MultiArray):
        """接收目标姿态 [yaw, pitch, roll]"""
        if len(msg.data) >= 3:
            self.target_yaw = msg.data[0]
            self.target_pitch = msg.data[1]
            self.target_roll = msg.data[2]
            self.has_target = True
        else:
            self.get_logger().warn(f'Target pose needs 3 values, got {len(msg.data)}')

    def joint_callback(self, msg: JointState):
        """解析 /joint_states 中的 pitch/roll 电机状态"""
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            if name == 'yaw':
                self.yaw_pos = pos
            elif name == 'pitch':
                self.theta1 = pos
                self.theta1_dot = vel
            elif name == 'roll':
                self.theta2 = pos
                self.theta2_dot = vel

        # 按节流发布控制命令 (250Hz 输入 → 每帧都算)
        self.compute_and_publish()

    def compute_and_publish(self):
        """执行力矩解算并发布"""
        if not self.has_target:
            return

        gains = HeadControlGains(
            kp_pitch=self.get_parameter('kp_pitch').value,
            kd_pitch=self.get_parameter('kd_pitch').value,
            kp_roll=self.get_parameter('kp_roll').value,
            kd_roll=self.get_parameter('kd_roll').value,
            max_joint_torque=self.get_parameter('max_joint_torque').value,
            max_motor_torque=self.get_parameter('max_motor_torque').value,
        )

        with suppress_stdout():
            result = compute_motor_torque_command(
                self.model,
                self.target_pitch,
                self.target_roll,
                self.theta1,
                self.theta2,
                self.theta1_dot,
                self.theta2_dot,
                gains,
                initial_pitch=self.last_pitch,
                initial_roll=self.last_roll,
                yaw=self.yaw_pos,
            )

        if result['error_state'] != 0:
            self.get_logger().warn(
                f'Torque error (state={result["error_state"]}) '
                f't1={self.theta1:.4f} t2={self.theta2:.4f}',
                throttle_duration_sec=1.0)
            return

        # 更新 FK 初值
        self.last_pitch = result['pitch']
        self.last_roll = result['roll']

        # /t002_controller/command: [yaw_target, pitch_cmd, roll_cmd]
        # 这里发送力矩值 (控制器需配置 Kp=1, Kd=0 透传)
        cmd = Float64MultiArray()
        cmd.data = [
            self.target_yaw,
            result['theta1_torque'],
            result['theta2_torque'],
        ]
        self.pub_cmd.publish(cmd)

        # 调试信息
        debug = Float64MultiArray()
        debug.data = [
            result['pitch'], result['roll'],           # 实际姿态
            result['pitch_error'], result['roll_error'],  # 姿态误差
            result['tau_pitch'], result['tau_roll'],     # 平台力矩
            result['theta1_torque'], result['theta2_torque'],  # 电机力矩
        ]
        self.pub_debug.publish(debug)


def main(args=None):
    rclpy.init(args=args)
    node = TorqueControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
