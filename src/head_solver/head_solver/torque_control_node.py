#!/usr/bin/env python3
"""全关节控制节点: 脖子力矩解算 + 底盘运动学"""
import os, sys, math
from contextlib import contextmanager
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, Imu

@contextmanager
def suppress_stdout():
    fd = os.open(os.devnull, os.O_WRONLY)
    old = os.dup(1)
    os.dup2(fd, 1)
    os.close(fd)
    try: yield
    finally:
        os.dup2(old, 1)
        os.close(old)

with suppress_stdout():
    from head_solver.head_solver import (create_model, compute_motor_torque_command, HeadControlGains)

@dataclass
class Joint:
    pos: float = 0.0
    vel: float = 0.0

@dataclass
class ImuState:
    """IMU 数据"""
    ori_w: float = 0.0  # 四元数 w
    ori_x: float = 0.0
    ori_y: float = 0.0
    ori_z: float = 0.0
    ang_vel_x: float = 0.0  # 角速度 rad/s
    ang_vel_y: float = 0.0
    ang_vel_z: float = 0.0
    lin_acc_x: float = 0.0  # 线加速度 m/s²
    lin_acc_y: float = 0.0
    lin_acc_z: float = 0.0
    roll:      float = 0.0
    yaw:       float = 0.0
    pitch:     float = 0.0

@dataclass
class RobotState:
    waist_joint            = Joint()
    neck_yaw_joint         = Joint()
    neck_pitch_joint       = Joint()
    neck_roll_joint        = Joint()
    wheel_left_yaw_joint   = Joint()
    wheel_left_roll_joint  = Joint()
    wheel_right_yaw_joint  = Joint()
    wheel_right_roll_joint = Joint()
    waist_joint_tar            = 0.0
    neck_yaw_joint_tar         = 0.0
    neck_pitch_joint_tar       = 0.0
    neck_roll_joint_tar        = 0.0
    wheel_left_yaw_joint_tar   = 0.0
    wheel_left_roll_joint_tar  = 0.0
    wheel_right_yaw_joint_tar  = 0.0
    wheel_right_roll_joint_tar = 0.0
    imu = ImuState()

class TorqueControlNode(Node):
    def __init__(self):
        super().__init__('torque_control_node')
        with suppress_stdout():
            self.model = create_model()
        self.robot = RobotState()
        self.declare_parameter('kp_pitch', 0.75)
        self.declare_parameter('kd_pitch', 0.005)
        self.declare_parameter('kp_roll', 0.75)
        self.declare_parameter('kd_roll', 0.005)
        self.declare_parameter('max_joint_torque', 2.0)
        self.declare_parameter('max_motor_torque', 1.5)
        self.declare_parameter('wheel_base', 0.44553)
        self.declare_parameter('wheel_radius', 0.125)
        self.target_waist    = 0.0
        self.target_neck_yaw = 0.0
        self.target_pitch    = 0.0
        self.target_roll     = 0.0
        self.target_vx       = 0.0
        self.target_vy       = 0.0
        self.target_wz       = 0.0
        self.last_pitch = 0.0
        self.last_roll  = 0.0
        self.sub_joints = self.create_subscription(
            JointState, '/joint_states', self.joint_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST))
        self.sub_target = self.create_subscription(Float64MultiArray, '/target_pose', self.target_callback, 10)
        self.sub_imu = self.create_subscription(
            Imu, '/imu', self.imu_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST))
        self.pub_cmd   = self.create_publisher(Float64MultiArray, '/t002_controller/command', 10)
        self.pub_debug = self.create_publisher(Float64MultiArray, '/head_solver/torque_debug', 10)

    def target_callback(self, msg):
        if len(msg.data) < 7: return
        self.target_waist    = msg.data[0]
        self.target_neck_yaw = msg.data[1]
        self.target_pitch    = msg.data[2]
        self.target_roll     = msg.data[3]
        self.target_vx       = msg.data[4]
        self.target_vy       = msg.data[5]
        self.target_wz       = msg.data[6]

    def imu_callback(self, msg: Imu):
        self.robot.imu.ori_w = msg.orientation.w
        self.robot.imu.ori_x = msg.orientation.x
        self.robot.imu.ori_y = msg.orientation.y
        self.robot.imu.ori_z = msg.orientation.z
        self.robot.imu.ang_vel_x = msg.angular_velocity.x
        self.robot.imu.ang_vel_y = msg.angular_velocity.y
        self.robot.imu.ang_vel_z = msg.angular_velocity.z
        self.robot.imu.lin_acc_x = msg.linear_acceleration.x
        self.robot.imu.lin_acc_y = msg.linear_acceleration.y
        self.robot.imu.lin_acc_z = msg.linear_acceleration.z

        w, x, y, z = self.robot.imu.ori_w, self.robot.imu.ori_x, self.robot.imu.ori_y, self.robot.imu.ori_z

        # roll
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # pitch
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        # yaw
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # 3. 赋值到 robot.imu
        self.robot.imu.roll  = roll
        self.robot.imu.pitch = pitch
        self.robot.imu.yaw   = yaw

        # self.get_logger().info(
        # f'IMU: roll={self.robot.imu.roll:.3f} pitch={self.robot.imu.pitch:.3f} yaw={self.robot.imu.yaw:.3f} rad',
        # throttle_duration_sec=0.5)

    def joint_callback(self, msg):
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            pos = self.normalize_angle_pi(pos)
            if   name == 'waist_joint':
                self.robot.waist_joint.pos = pos; self.robot.waist_joint.vel = vel
            elif name == 'neck_yaw_joint':
                self.robot.neck_yaw_joint.pos = pos; self.robot.neck_yaw_joint.vel = vel
            elif name == 'neck_pitch_joint':
                self.robot.neck_pitch_joint.pos = pos; self.robot.neck_pitch_joint.vel = vel
            elif name == 'neck_roll_joint':
                self.robot.neck_roll_joint.pos = pos; self.robot.neck_roll_joint.vel = vel
            elif name == 'wheel_left_yaw_joint':
                self.robot.wheel_left_yaw_joint.pos = pos; self.robot.wheel_left_yaw_joint.vel = vel
            elif name == 'wheel_left_roll_joint':
                self.robot.wheel_left_roll_joint.pos = pos; self.robot.wheel_left_roll_joint.vel = vel
            elif name == 'wheel_right_yaw_joint':
                self.robot.wheel_right_yaw_joint.pos = pos; self.robot.wheel_right_yaw_joint.vel = vel
            elif name == 'wheel_right_roll_joint':
                self.robot.wheel_right_roll_joint.pos = pos; self.robot.wheel_right_roll_joint.vel = vel
        self.compute_and_publish()

    def chassis_control(self):
        waist  = self.robot.waist_joint.pos
        vx, vy, wz = self.target_vx, self.target_vy, self.target_wz
        half   = self.get_parameter('wheel_base').value / 2.0
        radius = self.get_parameter('wheel_radius').value
        cos_w = math.cos(waist); sin_w = math.sin(waist)
        vx_c =  vx * cos_w - vy * sin_w
        vy_c =  vx * sin_w + vy * cos_w #？
        def steer(RA, RB, cur_pos):
            tar = math.atan2(RB, RA); vel = math.hypot(RA, RB) / radius
            if abs(cur_pos - tar) > math.pi / 2.0: tar += math.pi; vel = -vel
            tar = math.atan2(math.sin(tar), math.cos(tar))
            if abs(cur_pos - tar) < 0.6: vel *= 0.5
            return tar, vel
        self.robot.wheel_right_yaw_joint_tar,  self.robot.wheel_right_roll_joint_tar = \
            steer(vx_c + wz * half, vy_c, self.robot.wheel_right_yaw_joint.pos)
        self.robot.wheel_left_yaw_joint_tar,   self.robot.wheel_left_roll_joint_tar = \
            steer(vx_c - wz * half, vy_c, self.robot.wheel_left_yaw_joint.pos)
        self.robot.wheel_left_roll_joint_tar = self.robot.wheel_left_roll_joint_tar #这里反一下是为了把坐标系转换到上面
        self.robot.wheel_right_roll_joint_tar = -self.robot.wheel_right_roll_joint_tar
        
        

    # ── 脖子控制 ──
    def neck_control(self):
        # ---- 真机模式: FK + PD + Jacobian (真机部署时注释掉) ----
        gains = HeadControlGains(
            kp_pitch=self.get_parameter('kp_pitch').value,
            kd_pitch=self.get_parameter('kd_pitch').value,
            kp_roll=self.get_parameter('kp_roll').value,
            kd_roll=self.get_parameter('kd_roll').value,
            max_joint_torque=self.get_parameter('max_joint_torque').value,
            max_motor_torque=self.get_parameter('max_motor_torque').value)
        with suppress_stdout():
            result = compute_motor_torque_command(
                self.model, self.target_pitch, self.target_roll,
                self.robot.neck_pitch_joint.pos, self.robot.neck_roll_joint.pos,
                self.robot.neck_pitch_joint.vel, self.robot.neck_roll_joint.vel,
                gains, initial_pitch=self.last_pitch, initial_roll=self.last_roll)
        if result['error_state'] == 0:
            self.robot.neck_pitch_joint_tar = result['theta1_torque']
            self.robot.neck_roll_joint_tar  = result['theta2_torque']
            self.last_pitch = result['pitch']
            self.last_roll  = result['roll']
        # ---- 真机模式结束 ----

        # ---- 仿真模式: 直接 PD ----
        # kp_p = self.get_parameter('kp_pitch').value
        # kd_p = self.get_parameter('kd_pitch').value
        # kp_r = self.get_parameter('kp_roll').value
        # kd_r = self.get_parameter('kd_roll').value

        # self.robot.neck_pitch_joint_tar = (
        #     kp_p * (self.target_pitch - self.robot.neck_pitch_joint.pos)
        #     - kd_p * self.robot.neck_pitch_joint.vel)
        # self.robot.neck_roll_joint_tar  = (
        #     kp_r * (self.target_roll - self.robot.neck_roll_joint.pos)
        #     - kd_r * self.robot.neck_roll_joint.vel)
        # ---- 仿真模式结束 ----

        self.robot.neck_yaw_joint_tar = self.target_neck_yaw
        self.get_logger().info(
        f'neck: pitch={self.last_pitch:.3f} roll={self.last_roll:.3f} , rad',throttle_duration_sec=0.5)
    
    def gimbel_control(self):
        #使用陀螺仪控制
        error_pos = self.target_waist - self.robot.imu.yaw
        error_pos = self.normalize_angle_pi(error_pos)
        self.robot.waist_joint_tar = self.normalize_angle_pi(self.robot.waist_joint.pos + error_pos)

        #使用电机角度控制
        #self.robot.waist_joint_tar    = self.target_waist

        # self.get_logger().info(
        # f'gimbel: error_pos={error_pos:.3f} waist_joint_tar={self.robot.waist_joint_tar:.3f} target_waist={self.target_waist:.3f}, rad',throttle_duration_sec=0.5)

    @staticmethod
    def normalize_angle_pi(angle: float) -> float:
        """
        将角度归一化到 [-π, π] 区间。
        使用 atan2(sin, cos) 方法，数值稳定且避免分支。
        """
        return math.atan2(math.sin(angle), math.cos(angle))

    # ── 解算 + 发布 ──
    def compute_and_publish(self):
        self.neck_control()
        self.gimbel_control()
        self.chassis_control()
        cmd = Float64MultiArray()
        cmd.data = [self.robot.waist_joint_tar, self.robot.neck_yaw_joint_tar,
                    self.robot.neck_pitch_joint_tar, self.robot.neck_roll_joint_tar,
                    self.robot.wheel_left_yaw_joint_tar, self.robot.wheel_left_roll_joint_tar,
                    self.robot.wheel_right_yaw_joint_tar, self.robot.wheel_right_roll_joint_tar]
        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TorqueControlNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
