#!/usr/bin/env python3
"""
从标定数据 CSV 中提取摩擦系数: Ts, Tc, Bv

输入: pitch_friction_ramp.csv 或 pitch_friction_steady.csv
输出: 拟合结果 (控制台 + 系数文件)

拟合模型: T_friction = Tc + Bv * |ω|   (动摩擦)
          T_friction = Ts            (静摩擦, ω≈0)
"""

import csv
import sys
import os
import math
import argparse


def load_ramp_csv(path):
    """加载斜坡测试 CSV, 返回 (torque[], speed[], pos[])"""
    torques, speeds, positions = [], [], []
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            torques.append(float(row['torque_cmd_nm']))
            speeds.append(float(row['speed_rad_s']))
            positions.append(float(row['position_rad']))
    return torques, speeds, positions


def load_steady_csv(path):
    """加载稳态测试 CSV, 返回 (torque[], speed_avg[], speed_std[])"""
    torques, speeds, stds = [], [], []
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            torques.append(float(row['torque_cmd_nm']))
            speeds.append(float(row['speed_avg_rad_s']))
            stds.append(float(row['speed_std_rad_s']))
    return torques, speeds, stds


def estimate_ts_from_ramp(torques, speeds, speed_threshold=0.05):
    """
    从斜坡数据估算静摩擦系数 Ts
    找到转速首次超过阈值时的力矩指令
    """
    for i, (tq, spd) in enumerate(zip(torques, speeds)):
        if abs(spd) > speed_threshold:
            return tq, i
    return None, -1


def linear_fit(x, y):
    """最小二乘线性拟合 y = a + b*x, 返回 (intercept, slope, r_squared)"""
    n = len(x)
    if n < 2:
        return 0, 0, 0
    
    sx = sum(x)
    sy = sum(y)
    sxy = sum(xi * yi for xi, yi in zip(x, y))
    sxx = sum(xi * xi for xi in x)
    
    slope = (n * sxy - sx * sy) / (n * sxx - sx * sx) if (n * sxx - sx * sx) != 0 else 0
    intercept = (sy - slope * sx) / n
    
    # R²
    y_mean = sy / n
    ss_res = sum((yi - (intercept + slope * xi))**2 for xi, yi in zip(x, y))
    ss_tot = sum((yi - y_mean)**2 for yi in y)
    r_sq = 1 - ss_res / ss_tot if ss_tot != 0 else 0
    
    return intercept, slope, r_sq


def fit_from_ramp(torques, speeds, speed_threshold=0.05):
    """
    从斜坡数据提取摩擦系数:
    - Ts: 首次运动时的力矩
    - Tc + Bv*ω: 从运动段数据线性拟合
    """
    Ts, ts_idx = estimate_ts_from_ramp(torques, speeds, speed_threshold)
    
    # 取运动段数据 (去除静止段)
    if ts_idx is None or ts_idx >= len(torques):
        print("⚠ 电机在斜坡过程中未检测到运动，尝试使用全部数据...")
        ts_idx = 0
    
    # 从 Ts 之后 (电机启动) 取数据, 使用 |speed| > threshold 的点
    x, y = [], []
    for tq, spd in zip(torques[ts_idx:], speeds[ts_idx:]):
        if abs(spd) > speed_threshold:
            x.append(abs(spd))  # |ω|
            y.append(tq)         # T_cmd
    
    if len(x) < 5:
        print(f"⚠ 运动段数据点太少 ({len(x)}), 无法可靠拟合")
        return Ts, 0, 0, 0
    
    Tc, Bv, r2 = linear_fit(x, y)
    return Ts, Tc, Bv, r2


def fit_from_steady(torques, speeds):
    """
    从稳态数据提取: T = Tc + Bv * |ω|
    """
    x = [abs(s) for s in speeds]  # |ω|
    y = torques                    # T_cmd
    
    Tc, Bv, r2 = linear_fit(x, y)
    return Tc, Bv, r2


def print_results(Ts, Tc, Bv, r2, source):
    print(f"\n{'='*60}")
    print(f"  摩擦系数拟合结果 (数据源: {source})")
    print(f"{'='*60}")
    if Ts is not None:
        print(f"  静摩擦系数  Ts = {Ts:.4f} N·m")
    else:
        print(f"  静摩擦系数  Ts = (未测, 假定 = Tc)")
        Ts = Tc
    print(f"  库仑摩擦    Tc = {Tc:.4f} N·m")
    print(f"  粘性阻尼    Bv = {Bv:.6f} N·m·s/rad")
    print(f"  拟合优度    R² = {r2:.4f}")
    print(f"{'='*60}")
    
    # 补偿公式
    print(f"\n  补偿公式:")
    print(f"    |ω| ≤ ε:   T_comp = {Ts:.4f} * sign(T_des)")
    print(f"    |ω| > ε:   T_comp = {Tc:.4f} * sign(ω) + {Bv:.6f} * ω")
    print()
    
    return Ts, Tc, Bv


def main():
    parser = argparse.ArgumentParser(description='摩擦系数拟合')
    parser.add_argument('csv_file', nargs='?', help='标定 CSV 文件路径')
    parser.add_argument('--ramp', default=None, help='斜坡 CSV (默认自动检测)')
    parser.add_argument('--steady', default=None, help='稳态 CSV (默认自动检测)')
    parser.add_argument('--threshold', type=float, default=0.05,
                        help='速度阈值 rad/s (默认: 0.05)')
    parser.add_argument('--ts-assume-tc', action='store_true', default=True,
                        help='如果测不到 Ts, 令 Ts=Tc')
    args = parser.parse_args()
    
    # 自动检测文件类型
    if args.ramp:
        ramp_path = args.ramp
    elif args.csv_file and 'ramp' in args.csv_file.lower():
        ramp_path = args.csv_file
    else:
        ramp_path = os.path.join(os.path.dirname(__file__), 'pitch_friction_ramp.csv')
    
    if args.steady:
        steady_path = args.steady
    elif args.csv_file and 'steady' in args.csv_file.lower():
        steady_path = args.csv_file
    else:
        steady_path = os.path.join(os.path.dirname(__file__), 'pitch_friction_steady.csv')
    
    # 优先稳态数据 (更可靠)
    if os.path.exists(steady_path):
        print(f"使用稳态数据: {steady_path}")
        torques, speeds, stds = load_steady_csv(steady_path)
        Tc, Bv, r2 = fit_from_steady(torques, speeds)
        Ts = Tc  # 假定静摩擦=动摩擦
        print_results(Ts, Tc, Bv, r2, "稳态测试")
    elif os.path.exists(ramp_path):
        print(f"使用斜坡数据: {ramp_path}")
        torques, speeds, pos = load_ramp_csv(ramp_path)
        Ts, Tc, Bv, r2 = fit_from_ramp(torques, speeds, args.threshold)
        if Ts is None:
            Ts = Tc
        print_results(Ts, Tc, Bv, r2, "斜坡测试")
    else:
        print(f"未找到数据文件!")
        print(f"  期望位置: {ramp_path} 或 {steady_path}")
        sys.exit(1)


if __name__ == '__main__':
    main()
