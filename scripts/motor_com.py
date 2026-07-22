#!/usr/bin/env python3
"""
计算重力补偿项 joint_comp 模长最大时的电机力矩，
以及两个电机在整个工作范围内的最大力矩（绝对值）。
独立脚本，自动添加 head_solver 路径。
"""

import os
import sys
import math
import numpy as np

# ---------- 自动添加 head_solver 路径 ----------
# 获取脚本所在目录，并推断项目根目录
script_dir = os.path.dirname(os.path.abspath(__file__))
# 如果脚本在 scripts/ 子目录，则项目根目录为上级；否则认为就在根目录
if os.path.basename(script_dir) == 'scripts':
    project_root = os.path.dirname(script_dir)
else:
    project_root = script_dir

# 可能的 head_solver 路径（优先使用 install 后的路径）
possible_paths = [
    os.path.join(project_root, 'install', 'head_solver', 'lib', 'python3.10', 'site-packages'),
    os.path.join(project_root, 'src', 'head_solver'),
    os.path.join(project_root, '..', 'install', 'head_solver', 'lib', 'python3.10', 'site-packages'),  # 如果脚本在子目录
]
head_path = None
for p in possible_paths:
    if os.path.exists(p) and os.path.isdir(p):
        head_path = p
        break

if head_path is None:
    print("Error: Could not find head_solver package.")
    print("Searched in:", possible_paths)
    sys.exit(1)

sys.path.insert(0, head_path)
# ---------------------------------------------

from head_solver.head_solver import create_model

DEG = math.pi / 180.0

def compute_joint_comp(r, p):
    """根据当前 roll(r) 和 pitch(p) (弧度) 计算 joint_comp = [comp_r, comp_p]"""
    comp_p = 0.185 * math.sin(r + 166*DEG) * math.cos(p - 70*DEG)
    comp_r = 0.071 * math.sin(r - 4*DEG)   * math.cos(p + 156*DEG)
    return np.array([comp_r, comp_p])

def main():
    # 1. 加载模型
    model = create_model()
    print("模型加载完成。")

    # 2. 定义工作范围（弧度）
    pitch_min, pitch_max = model.pitch_range[0], model.pitch_range[1]
    roll_min, roll_max = model.roll_range[0], model.roll_range[1]
    print(f"Pitch 范围: [{pitch_min*180/math.pi:.2f}°, {pitch_max*180/math.pi:.2f}°]")
    print(f"Roll  范围: [{roll_min*180/math.pi:.2f}°, {roll_max*180/math.pi:.2f}°]")

    # 3. 网格分辨率
    num_pitch = 200
    num_roll  = 200
    pitch_vals = np.linspace(pitch_min, pitch_max, num_pitch)
    roll_vals  = np.linspace(roll_min, roll_max, num_roll)

    max_norm = 0.0
    best_r = best_p = 0.0
    best_motor = np.zeros(2)
    max_motor1 = 0.0
    max_motor2 = 0.0

    # 4. 遍历网格
    for p in pitch_vals:
        for r in roll_vals:
            theta1, theta2, ik_err = model.ik(p, r)
            if ik_err != 0:
                continue
            Jc, jac_err = model.Jac(p, r, theta1, theta2)
            if jac_err != 0:
                continue

            joint_comp = compute_joint_comp(r, p)
            norm_jc = np.linalg.norm(joint_comp)

            try:
                motor_comp = np.linalg.solve(Jc.T, joint_comp)
            except np.linalg.LinAlgError:
                continue

            if norm_jc > max_norm:
                max_norm = norm_jc
                best_r, best_p = r, p
                best_motor = motor_comp.copy()

            if abs(motor_comp[0]) > max_motor1:
                max_motor1 = abs(motor_comp[0])
            if abs(motor_comp[1]) > max_motor2:
                max_motor2 = abs(motor_comp[1])

    # 5. 输出结果
    print("\n=== 补偿力矩最大时 ===")
    print(f"最大 joint_comp 模长: {max_norm:.6f}")
    print(f"对应姿态: pitch = {best_p*180/math.pi:.4f}°, roll = {best_r*180/math.pi:.4f}°")
    print(f"电机力矩: theta1_torque = {best_motor[0]:.6f}, theta2_torque = {best_motor[1]:.6f}")

    print("\n=== 整个工作范围内电机最大力矩（绝对值）===")
    print(f"电机1 (theta1) 最大力矩: {max_motor1:.6f}")
    print(f"电机2 (theta2) 最大力矩: {max_motor2:.6f}")

if __name__ == "__main__":
    main()