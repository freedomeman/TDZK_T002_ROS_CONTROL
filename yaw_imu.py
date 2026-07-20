#!/usr/bin/env python3
"""
IMU Yaw 漂移线性最小二乘拟合 (无 pandas 依赖)
输入：imu_yaw_log.csv (第一列 time_ms，第二列 yaw_deg)
输出：拟合斜率 k (°/s)，截距 b (°)，R²，残差图
"""

import csv
import numpy as np
import matplotlib.pyplot as plt

# ========== 1. 读取 CSV ==========
time_ms = []
yaw_deg = []
with open('imu_yaw_log.csv', 'r') as f:
    reader = csv.reader(f)
    header = next(reader)  # 跳过表头 "time_ms,yaw_deg"
    for row in reader:
        if len(row) >= 2:
            time_ms.append(float(row[0]))
            yaw_deg.append(float(row[1]))

# 转换为 numpy 数组
time_ms = np.array(time_ms)
yaw_deg = np.array(yaw_deg)

# ========== 2. 预处理：时间归一化为相对秒数 ==========
t0 = time_ms[0]
t_sec = (time_ms - t0) / 1000.0   # 单位：秒，起始时刻为0

# ========== 3. 线性最小二乘拟合 (次数=1) ==========
k, b = np.polyfit(t_sec, yaw_deg, 1)   # k 斜率, b 截距
y_fit = k * t_sec + b

# ========== 4. 计算拟合指标 ==========
residuals = yaw_deg - y_fit
rss = np.sum(residuals**2)                     # 残差平方和
tss = np.sum((yaw_deg - np.mean(yaw_deg))**2)  # 总平方和
r2 = 1 - rss / tss                             # 决定系数 R²

# ========== 5. 打印结果 ==========
print(f"拟合直线: yaw = {k:.6f} * t + {b:.6f}")
print(f"漂移率 k = {k:.6f} °/s")
print(f"初始角度 b = {b:.6f} °")
print(f"残差平方和 RSS = {rss:.2f}")
print(f"决定系数 R² = {r2:.6f}")

# ========== 6. 绘图 ==========
plt.figure(figsize=(14, 6))

# 6.1 原始数据与拟合直线 (每100个点采样显示)
plt.subplot(1, 2, 1)
plt.plot(t_sec[::100], yaw_deg[::100], 'b.', markersize=1, label='原始数据 (采样)')
plt.plot(t_sec, y_fit, 'r-', linewidth=2, label=f'拟合: y={k:.4f}t+{b:.2f}')
plt.xlabel('时间 (秒)')
plt.ylabel('Yaw 角度 (度)')
plt.title('Yaw 线性拟合')
plt.legend()
plt.grid(True)

# 6.2 残差分布
plt.subplot(1, 2, 2)
plt.plot(t_sec, residuals, 'g.', markersize=0.5)
plt.axhline(y=0, color='r', linestyle='--', linewidth=1)
plt.xlabel('时间 (秒)')
plt.ylabel('残差 (度)')
plt.title('残差分布 (线性拟合)')
plt.grid(True)

plt.tight_layout()
plt.savefig('yaw_fit_result.png', dpi=150)
plt.show()