#!/usr/bin/env python3
"""全关节控制节点: 脖子力矩解算 + 底盘运动学"""
import os, sys, math
import numpy as np
from contextlib import contextmanager
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, Imu
from face_msgs.msg import FaceTarget


# ============================================================
# 云台指向解算（纯数学函数，无 ROS 依赖）
# ============================================================
def calculate_target_angles(point_cam, current_roll, current_yaw, current_pitch):
    """三轴云台（Roll / Yaw / Pitch）高精度指向解算。

    根据目标点在**相机坐标系**下的三维坐标，以及云台当前三个姿态角，
    反算为使云台正前方（X 轴）精确指向该目标所需的**绝对目标角度**。

    ⚠️ 坐标系约定（调用前务必确认）：

      本函数期望的输入坐标系（与云台物理定义一致）：
        · X 轴 — 指向云台正前方 (Forward)
        · Y 轴 — 指向云台正左方 (Left)
        · Z 轴 — 指向云台正上方 (Up)

      如果上游视觉模块输出的是「视觉惯例坐标系」
      （X 向右, Y 向下, Z 向前），调用方**必须**在传入前
      手动映射为：

          point_cam = [Z_vis, -Y_vis, X_vis]   # 视觉惯例 → 本函数坐标系

    Args:
        point_cam: 目标在相机系下的坐标 [X, Y, Z]，单位 **米**。
                   相机坐标系方向与旋转坐标系完全相同（无旋转偏差）。
        current_roll:  当前 Roll  角（绕 X 轴），单位 **弧度**。
        current_yaw:   当前 Yaw   角（绕 Z 轴），单位 **弧度**。
        current_pitch: 当前 Pitch 角（绕 Y 轴），单位 **弧度**。

    Returns:
        (target_roll, target_yaw, target_pitch)
        单位均为弧度。target_roll 根据设计恒为 0.0。

    Raises:
        ValueError: point_cam 长度不为 3。

    算法步骤：
        1. 视差补偿 — 相机在 Rot 系偏移 X+56mm, Z+125mm：
           P_rot = (X_cam+0.056, Y_cam, Z_cam+0.125)
        2. 坐标系旋转 — 利用当前云台角将 P_rot 变换到基座系：
           P_base = Rz(yaw) · Ry(pitch) · Rx(roll) · P_rot
           （Rx/Ry/Rz 均为标准旋转矩阵，与 FK 约定一致）
        3. 反算指向角：
           target_yaw   = atan2(Y_base, X_base)
           target_pitch = atan2(Z_base, sqrt(X_base² + Y_base²))

    边界处理：
        · 水平距离 < 1e-12 时，pitch 根据 Z_base 正负返回 ±π/2。
        · 不施加任何物理限幅，Yaw 输出纯数学解 [-π, π)。
    """
    if len(point_cam) != 3:
        raise ValueError(f'point_cam 长度必须为 3，实际: {len(point_cam)}')

    # ── 1. 视差补偿：相机在 Rot 系偏移 X+56mm, Z+125mm ──
    X_rot = float(point_cam[0]) + 0.06859
    Y_rot = float(point_cam[1])
    Z_rot = float(point_cam[2]) + 0.125

    # ── 2. 旋转到基座系: P_base = Rz(yaw)·Ry(pitch)·Rx(roll)·P_rot ──
    cr = math.cos(current_roll)
    sr = math.sin(current_roll)
    cp = math.cos(current_pitch)
    sp = math.sin(current_pitch)
    cy = math.cos(current_yaw)
    sy = math.sin(current_yaw)

    # Rx(roll) — 绕 X 轴旋转（正方向：Y→Z，与 FK 一致）
    x1 = X_rot
    y1 = Y_rot * cr - Z_rot * sr
    z1 = Y_rot * sr + Z_rot * cr

    # Ry(pitch) — 标准形式（匹配 FK: X→-Z，抬头为负）
    x2 = x1 * cp + z1 * sp
    y2 = y1
    z2 = -x1 * sp + z1 * cp

    # Rz(yaw) — 绕 Z 轴旋转（正方向：X→Y，左转为正）
    X_base = x2 * cy - y2 * sy
    Y_base = x2 * sy + y2 * cy
    Z_base = z2

    # ── 3. 反算目标角度 ──
    target_roll = 0.0
    target_yaw = math.atan2(Y_base, X_base)

    h_dist = math.hypot(X_base, Y_base)
    if h_dist < 1e-12:
        target_pitch = math.copysign(math.pi / 2.0, Z_base)
    else:
        # 精确 pitch：补偿相机在旋转中心 Z+125mm 的偏移
        d = math.hypot(h_dist, Z_base)  # 旋转中心到人脸的距离
        if d <= 0.125:
            target_pitch = math.copysign(math.pi / 2.0, Z_base)
        else:
            target_pitch = math.asin(0.125 / d) - math.atan2(Z_base, h_dist)

    return (target_roll, target_yaw, target_pitch, (X_base, Y_base, Z_base))


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
        self.declare_parameter('kp_pitch', 1.5)
        self.declare_parameter('kd_pitch', 0.1)
        self.declare_parameter('kp_roll', 1.5)
        self.declare_parameter('kd_roll', 0.1)
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
        self.filt_x = None  # 相机坐标 EMA 滤波状态
        self.filt_y = None
        self.filt_z = None
        self.sub_joints = self.create_subscription(
            JointState, '/joint_states', self.joint_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST))
        self.sub_target = self.create_subscription(Float64MultiArray, '/target_pose', self.target_callback, 10)
        self.sub_imu = self.create_subscription(
            Imu, '/imu', self.imu_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST))
        self.sub_face = self.create_subscription(
            FaceTarget, '/face/target', self.face_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST))
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

    def face_callback(self, msg: FaceTarget):
        """人脸坐标回调：调用指向解算并打印结果。"""
        if not msg.has_target:
            return

        # EMA 滤波相机坐标（alpha=0.4，滤除头部运动时的测量噪声）
        alpha = 0.4
        if self.filt_x is None:
            self.filt_x = msg.center.x
            self.filt_y = msg.center.y
            self.filt_z = msg.center.z
        else:
            self.filt_x = alpha * msg.center.x + (1 - alpha) * self.filt_x
            self.filt_y = alpha * msg.center.y + (1 - alpha) * self.filt_y
            self.filt_z = alpha * msg.center.z + (1 - alpha) * self.filt_z

        point_cam = [self.filt_x, self.filt_y, self.filt_z]
        # 注意: self.last_pitch 正=抬头(用户约定), 函数内正=低头(FK约定), 需取反
        roll, yaw, pitch_fk, (X_b, Y_b, Z_b) = calculate_target_angles(
            point_cam,
            self.last_roll,
            self.robot.neck_yaw_joint.pos,
            -self.last_pitch,
        )
        pitch = -pitch_fk  # 转回用户约定: 正=抬头

        # 限位 ±0.3 rad
        roll  = 0.0
        pitch = max(-0.3, min(0.3, pitch))

        # setpoint 限速（防大阶跃超调振荡）
        max_rate_pitch = 0.8   # rad/s
        max_rate_yaw   = 1.2   # rad/s
        dt = 0.03              # face_callback 约 30Hz
        # pitch
        err_p = pitch - self.target_pitch
        step_p = max(-max_rate_pitch * dt, min(max_rate_pitch * dt, err_p))
        self.target_pitch += step_p
        # yaw
        err_y = yaw - self.target_neck_yaw
        step_y = max(-max_rate_yaw * dt, min(max_rate_yaw * dt, err_y))
        self.target_neck_yaw += step_y
        # roll
        self.target_roll = roll

        # error_p = (pitch - self.last_pitch)
        # error_y = (yaw - self.robot.neck_yaw_joint.pos)

        # if -0.05 <= error_p <= 0.05:
        #     error_p = 0
        # else:
        #     error_p = error_p/5

        # if -0.05 <= error_y <= 0.05:
        #     error_y = 0
        # else:
        #     error_y = error_y/5
        

        # self.target_neck_yaw = yaw   #self.target_neck_yaw + error_y
        # self.target_pitch    = pitch #self.target_pitch + error_p
        # self.target_roll     = roll
        self.get_logger().info(
            f'🎯 人脸指向解算: '
            f'pitch_now={self.last_pitch:.3f} '
            f'roll_now={self.last_roll:.3f} '
            f'yaw_tar={yaw:.3f} '
            f'pitch_tar={pitch:.3f} rad '
            f'P_base=({X_b:.3f},{Y_b:.3f},{Z_b:.3f}) '
            f'P_cam=({msg.center.x:.3f},{msg.center.y:.3f},{msg.center.z:.3f})',
            throttle_duration_sec=0.5,
        )

        # # ── 写调试日志到文件 (face 跟踪) ──
        # _debug_log_path = '/home/tuf/Doc/TDZK_T002_ROS_CONTROL/face_pointing_debug.csv'
        # _debug_header = (
        #     'time_ms,cur_roll,cur_pitch,neck_yaw,neck_pitch_joint,'
        #     'X_cam,Y_cam,Z_cam,X_base,Y_base,Z_base,yaw_tar,pitch_tar\n'
        # )
        # if not hasattr(self, '_debug_fp'):
        #     import os
        #     first = not os.path.exists(_debug_log_path)
        #     self._debug_fp = open(_debug_log_path, 'a')
        #     if first:
        #         self._debug_fp.write(_debug_header)
        # ts_ms = int(self.get_clock().now().nanoseconds / 1e6)
        # self._debug_fp.write(
        #     f'{ts_ms},{self.last_roll:.6f},{self.last_pitch:.6f},'
        #     f'{self.robot.neck_yaw_joint.pos:.6f},{self.robot.neck_pitch_joint.pos:.6f},'
        #     f'{msg.center.x:.6f},{msg.center.y:.6f},{msg.center.z:.6f},'
        #     f'{X_b:.6f},{Y_b:.6f},{Z_b:.6f},'
        #     f'{yaw:.6f},{pitch:.6f}\n'
        # )
        # self._debug_fp.flush()

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
        # 始终更新当前姿态（即使 FK 不收敛，近似值也可用）
        self.last_pitch = result['pitch']
        self.last_roll  = result['roll']

        # 重力补偿 (始终执行)
        DEG = math.pi / 180.0
        r, p = self.last_roll, self.last_pitch
        comp_p = 0.185 * math.sin(r + 166*DEG) * math.cos(p - 70*DEG)
        comp_r = 0.071 * math.sin(r - 4*DEG)   * math.cos(p + 156*DEG)
        joint_comp = np.array([comp_r, comp_p])
        motor_comp = np.linalg.solve(result['jacobian'].T, joint_comp)
        self.robot.neck_pitch_joint_tar = motor_comp[0]*0.8   + result['theta1_torque'] #这里参数不一样是因为电机的特性原因
        self.robot.neck_roll_joint_tar  = motor_comp[1]  + result['theta2_torque']
        # ---- 真机模式结束 ----

        # ── 重力标定数据采集 ──
        _calib_path = '/home/tuf/Doc/TDZK_T002_ROS_CONTROL/gravity_calib.csv'
        _calib_header = 'time_ms,target_pitch,target_roll,fk_pitch,fk_roll,tau_pitch,tau_roll\n'
        if not hasattr(self, '_calib_fp'):
            import os
            first = not os.path.exists(_calib_path)
            self._calib_fp = open(_calib_path, 'a')
            if first:
                self._calib_fp.write(_calib_header)
        ts_ms = int(self.get_clock().now().nanoseconds / 1e6)
        self._calib_fp.write(
            f'{ts_ms},{self.target_pitch:.6f},{self.target_roll:.6f},'
            f'{self.last_pitch:.6f},{self.last_roll:.6f},'
            f'{result["tau_pitch"]:.6f},{result["tau_roll"]:.6f}\n'
        )
        self._calib_fp.flush()

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
        # self.get_logger().info(
        # f'neck: pitch={self.last_pitch:.3f} roll={self.last_roll:.3f} , rad',throttle_duration_sec=0.5)
    
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




