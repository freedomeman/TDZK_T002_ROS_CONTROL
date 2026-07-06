#!/usr/bin/env python3
"""
pitch 电机摩擦系数标定脚本 (STW / GDZ 协议)

通信协议: GDZ 自定义 CAN V3.07
- CAN 接口: can0
- 电机 Dev_addr: 1
- MIT 运控帧 CAN ID: 0x400 | 1 = 0x401
- 反馈帧 CAN ID: 0x01 (0xF1 实时数据)
- 参数: PosMax=95.5rad, SpdMax=45rad/s, TauMax=28Nm

测量方法:
  1. 力矩斜坡 (0→3Nm, 15s): 连续记录力矩+转速, 从中提取静摩擦 Ts 和动摩擦数据
  2. 多转速稳态: 多个恒定力矩点, 记录稳态转速, 用于线性拟合 Tc + Bv*ω

输出: pitch_friction_data.csv (时间戳, 指令力矩Nm, 反馈转速rad/s, 反馈位置rad)
"""

import socket
import struct
import time
import csv
import os
import sys
import signal
import argparse
import fcntl

# ─── ioctl 常量 ────────────────────────────────────────
SIOCGIFINDEX = 0x8933

# ─── 电机参数 ───────────────────────────────────────────
MOTOR_ID      = 1
CAN_IF        = "can0"
MIT_CAN_ID    = 0x400 | MOTOR_ID   # 0x401
RESP_CAN_ID   = MOTOR_ID           # 0x01

# 限制参数 (与 stw_motor_driver.cpp 中 kStwLimitParams[0] 一致)
POS_MAX  = 95.5   # rad
SPD_MAX  = 45.0   # rad/s
TAU_MAX  = 28.0   # N·m
OKP_MAX  = 500.0
OKD_MAX  = 5.0


# ─── 工具函数 ───────────────────────────────────────────
def float_to_uint(value, v_min, v_max, bits):
    """浮点数 → uint, 线性映射到 [0, 2^bits-1]"""
    ratio = (value - v_min) / (v_max - v_min)
    ratio = max(0.0, min(1.0, ratio))
    return int(ratio * ((1 << bits) - 1))


def uint_to_float(raw, v_min, v_max, bits):
    """uint → 浮点数, 线性逆映射"""
    ratio = raw / float((1 << bits) - 1)
    return ratio * (v_max - v_min) + v_min


def open_can(interface):
    """打开 CAN 套接字并绑定到接口"""
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    
    # 获取接口索引
    ifreq = struct.pack('16sI', interface.encode(), 0)
    result = fcntl.ioctl(sock, SIOCGIFINDEX, ifreq)
    ifindex = struct.unpack('16sI', result)[1]
    
    addr = struct.pack('HiLL', socket.AF_CAN, ifindex, 0, 0)
    sock.bind(addr)
    sock.settimeout(0.1)
    return sock


def close_can(sock):
    """关闭 CAN 套接字"""
    sock.close()


def send_mit_cmd(sock, torque_nm):
    """
    发送 MIT 运控帧 (纯力矩前馈, KP=KD=0, 位置=速度=0)
    torque_nm: 期望力矩 (N·m)
    """
    # 归一化力矩 [-TauMax, TauMax], 然后取负 (STW 左手→右手)
    f_t = -torque_nm
    f_t = max(-TAU_MAX, min(TAU_MAX, f_t))
    
    p_raw  = float_to_uint(0.0, -POS_MAX, POS_MAX, 16)
    v_raw  = float_to_uint(0.0, -SPD_MAX, SPD_MAX, 12)
    kp_raw = float_to_uint(0.0, 0.0, OKP_MAX, 12)
    kd_raw = float_to_uint(0.0, 0.0, OKD_MAX, 12)
    t_raw  = float_to_uint(f_t, -TAU_MAX, TAU_MAX, 12)
    
    data = bytearray(8)
    data[0] = (p_raw >> 8) & 0xFF
    data[1] = p_raw & 0xFF
    data[2] = (v_raw >> 4) & 0xFF
    data[3] = ((v_raw & 0x0F) << 4) | ((kp_raw >> 8) & 0x0F)
    data[4] = kp_raw & 0xFF
    data[5] = (kd_raw >> 4) & 0xFF
    data[6] = ((kd_raw & 0x0F) << 4) | ((t_raw >> 8) & 0x0F)
    data[7] = t_raw & 0xFF
    
    # 构建 CAN 帧: struct can_frame { canid_t can_id; __u8 can_dlc; __u8 __pad;
    #                               __u8 __res0; __u8 __res1; __u8 data[8]; }
    frame = struct.pack('IBB3x8s', MIT_CAN_ID, 8, 0, bytes(data))
    sock.send(frame)


def send_motor_free(sock):
    """发送电机释放命令 (0xCF)"""
    data = bytearray(8)
    data[0] = 0xCF
    frame = struct.pack('IBB3x8s', MOTOR_ID, 1, 0, bytes(data))
    sock.send(frame)


def parse_0xf1(data):
    """
    解析 0xF1 反馈帧, 返回 (pos_rad, spd_rad_per_s)
    参考 stw_motor_driver.cpp parse_0xf1_response
    """
    if len(data) < 5 or data[0] != 0xF1:
        return None, None
    
    pos_raw = (data[1] << 8) | data[2]
    spd_raw = (data[3] << 4) | ((data[4] & 0xF0) >> 4)
    
    # 逆映射 (左手坐标系, 取负)
    pos = -(pos_raw / 65535.0 * (2.0 * POS_MAX) - POS_MAX)
    spd = -(spd_raw / 4095.0 * (2.0 * SPD_MAX) - SPD_MAX)
    
    return pos, spd


def recv_feedback(sock, timeout=0.1):
    """接收一个 CAN 帧, 如果是 0xF1 反馈则解析"""
    try:
        sock.settimeout(timeout)
        frame_data = sock.recv(16)
        can_id, dlc, _ = struct.unpack('IBB3x', frame_data[:10])
        data = frame_data[10:10+dlc]
        
        if can_id == RESP_CAN_ID and len(data) > 0 and data[0] == 0xF1:
            return parse_0xf1(data)
        return None, None
    except socket.timeout:
        return None, None


def drain_rx(sock, duration=0.5):
    """清空接收缓冲区"""
    sock.settimeout(0.01)
    end = time.time() + duration
    while time.time() < end:
        try:
            sock.recv(16)
        except (socket.timeout, BlockingIOError):
            break


# ─── 标定实验 ───────────────────────────────────────────

def torque_ramp_test(sock, output_csv, torque_max=3.0, ramp_time=15.0, hz=100):
    """
    力矩斜坡测试: 从 0 线性增加到 torque_max, 持续记录
    用于提取静摩擦 Ts 和动摩擦数据点
    """
    period = 1.0 / hz
    steps = int(ramp_time * hz)
    
    print(f"\n{'='*60}")
    print(f"力矩斜坡测试: 0 -> {torque_max} Nm, {ramp_time}s, {hz}Hz")
    print(f"数据保存到: {output_csv}")
    print(f"{'='*60}")
    
    with open(output_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp', 'torque_cmd_nm', 'speed_rad_s', 'position_rad'])
        
        t0 = time.time()
        
        for i in range(steps):
            torque = torque_max * (i / max(steps - 1, 1))
            
            send_mit_cmd(sock, torque)
            
            pos, spd = recv_feedback(sock, timeout=0.01)
            if pos is None:
                pos, spd = 0.0, 0.0
            
            t_now = time.time() - t0
            writer.writerow([f"{t_now:.6f}", f"{torque:.4f}", f"{spd:.4f}", f"{pos:.4f}"])
            
            if i % 50 == 0:
                print(f"  t={t_now:5.1f}s torque={torque:6.3f}Nm speed={spd:+7.3f}rad/s pos={pos:+7.3f}rad")
            
            # 精确周期
            elapsed = time.time() - t0
            sleep_t = (i + 1) * period - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)
    
    send_mit_cmd(sock, 0.0)
    print(f"\n斜坡测试完成, 共 {steps} 个数据点")


def steady_state_test(sock, output_csv, torque_levels=None, hold_time=2.0, hz=100):
    """
    多转矩稳态测试: 对每个力矩值保持一段时间, 记录稳态转速均值
    用于 Tc + Bv*omega 线性拟合
    """
    if torque_levels is None:
        torque_levels = [0.2, 0.4, 0.6, 0.8, 1.0, 1.5, 2.0, 2.5, 3.0]
    
    print(f"\n{'='*60}")
    print(f"多转矩稳态测试: {len(torque_levels)} 个等级, 每级 {hold_time}s")
    print(f"力矩等级: {torque_levels}")
    print(f"数据保存到: {output_csv}")
    print(f"{'='*60}")
    
    period = 1.0 / hz
    
    with open(output_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['torque_cmd_nm', 'speed_avg_rad_s', 'speed_std_rad_s',
                          'duration_s', 'num_samples'])
        
        for tq in torque_levels:
            send_mit_cmd(sock, tq)
            time.sleep(0.5)  # 过渡段
            
            speeds = []
            t_start = time.time()
            while time.time() - t_start < hold_time:
                _, spd = recv_feedback(sock, timeout=0.01)
                if spd is not None:
                    speeds.append(spd)
                time.sleep(max(0.0, period - 0.005))
            
            if len(speeds) > 0:
                avg = sum(speeds) / len(speeds)
                var = sum((s - avg)**2 for s in speeds) / len(speeds)
                std = var ** 0.5
            else:
                avg, std = 0.0, 0.0
            
            writer.writerow([f"{tq:.4f}", f"{avg:.6f}", f"{std:.6f}",
                             f"{hold_time:.3f}", len(speeds)])
            print(f"  torque={tq:5.2f}Nm -> speed_avg={avg:+8.4f}rad/s +-{std:.4f} (n={len(speeds)})")
    
    send_mit_cmd(sock, 0.0)
    print(f"\n稳态测试完成")


def motor_enable(sock):
    """电机使能: 先发零力矩 MIT 帧激活运控模式"""
    print("使能电机 (MIT 运控模式)...")
    drain_rx(sock, 0.3)
    for _ in range(5):
        send_mit_cmd(sock, 0.0)
        time.sleep(0.02)
    time.sleep(0.5)
    print("  完成")


def motor_disable(sock):
    """电机释放"""
    print("释放电机...")
    send_mit_cmd(sock, 0.0)
    time.sleep(0.1)
    send_motor_free(sock)
    time.sleep(0.2)
    print("  完成")


# ─── 主程序 ───────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='Pitch 电机摩擦标定')
    parser.add_argument('--can', default=CAN_IF, help=f'CAN 接口名 (默认: {CAN_IF})')
    parser.add_argument('--output-dir', default=None, help='输出目录 (默认: 当前目录)')
    parser.add_argument('--mode', choices=['ramp', 'steady', 'both'], default='both',
                        help='测试模式')
    parser.add_argument('--torque-max', type=float, default=3.0,
                        help='斜坡测试最大力矩 Nm (默认: 3.0)')
    parser.add_argument('--ramp-time', type=float, default=15.0,
                        help='斜坡持续时间 s (默认: 15)')
    parser.add_argument('--hz', type=int, default=100, help='记录频率 Hz (默认: 100)')
    args = parser.parse_args()
    
    out_dir = args.output_dir or '.'
    os.makedirs(out_dir, exist_ok=True)
    
    ramp_csv = os.path.join(out_dir, 'pitch_friction_ramp.csv')
    steady_csv = os.path.join(out_dir, 'pitch_friction_steady.csv')
    
    print(f"\n打开 CAN 接口: {args.can}")
    sock = open_can(args.can)
    print(f"  已绑定 {args.can}")
    
    def safe_exit(signum, frame):
        print("\n\n中断! 释放电机...")
        send_mit_cmd(sock, 0.0)
        time.sleep(0.05)
        send_motor_free(sock)
        close_can(sock)
        print("已安全退出")
        sys.exit(0)
    
    signal.signal(signal.SIGINT, safe_exit)
    signal.signal(signal.SIGTERM, safe_exit)
    
    try:
        motor_enable(sock)
        
        if args.mode in ('ramp', 'both'):
            torque_ramp_test(sock, ramp_csv, args.torque_max, args.ramp_time, args.hz)
        
        if args.mode in ('steady', 'both'):
            steady_state_test(sock, steady_csv, hold_time=2.0, hz=args.hz)
        
    finally:
        motor_disable(sock)
        close_can(sock)
        print(f"\n全部完成, 数据文件:")
        if args.mode in ('ramp', 'both'):
            print(f"  斜坡: {ramp_csv}")
        if args.mode in ('steady', 'both'):
            print(f"  稳态: {steady_csv}")


if __name__ == '__main__':
    main()
