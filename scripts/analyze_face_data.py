#!/usr/bin/env python3
"""分析 face_track.csv 相机数据质量 (无第三方依赖)"""
import sys
import math

CSV_PATH = '/home/tuf/Doc/TDZK_T002_ROS_CONTROL/face_track.csv'
if len(sys.argv) > 1:
    CSV_PATH = sys.argv[1]

lines = open(CSV_PATH).readlines()
header = lines[0].strip().split(',')
col = {h: i for i, h in enumerate(header)}

raw = []
for l in lines[1:]:
    parts = l.strip().split(',')
    if len(parts) < len(header):
        continue
    raw.append([float(v) for v in parts])

n = len(raw)
if n == 0:
    print('没有数据')
    sys.exit(1)

t  = [r[col['time_ms']] for r in raw]
ht = [r[col['has_target']] for r in raw]
rx = [r[col['raw_x']] for r in raw]
ry = [r[col['raw_y']] for r in raw]
rz = [r[col['raw_z']] for r in raw]
fx = [r[col['filt_x']] for r in raw]
fy = [r[col['filt_y']] for r in raw]
fz = [r[col['filt_z']] for r in raw]
fvx = [r[col['filt_vx']] for r in raw]
fvy = [r[col['filt_vy']] for r in raw]
fvz = [r[col['filt_vz']] for r in raw]
ya = [r[col['point_yaw']] for r in raw]
pi = [r[col['point_pitch']] for r in raw]

def mean(v):
    return sum(v) / len(v)

def std(v):
    m = mean(v)
    return math.sqrt(sum((x - m)**2 for x in v) / len(v))

print(f'总行数: {n}')
print()

# 丢帧率
drop = sum(1 for x in ht if x == 0) / n
print(f'【丢帧率】 {drop:.1%}')
print()

# 帧率
dts = [(t[i] - t[i-1]) / 1000.0 for i in range(1, len(t))]
avg_dt = mean(dts) * 1000
max_dt = max(dts) * 1000
fps = 1000.0 / avg_dt if avg_dt > 0 else 0
print(f'【帧率】 均值={avg_dt:.0f}ms  最大={max_dt:.0f}ms  有效帧率≈{fps:.0f}Hz')
print()

# 静止段噪声
dya = [abs(ya[i] - ya[i-1]) for i in range(1, len(ya))]
dpi = [abs(pi[i] - pi[i-1]) for i in range(1, len(pi))]
static_idx = [i for i in range(len(dya)) if dya[i] < 0.005 and dpi[i] < 0.005]
if len(static_idx) > 5:
    print(f'【静止段噪声】({len(static_idx)}帧)')
    for cname, cdata in [('raw_x', rx), ('raw_y', ry), ('raw_z', rz)]:
        vals = [cdata[i] for i in static_idx]
        print(f'  {cname}: std={std(vals):.4f}  min={min(vals):.3f}  max={max(vals):.3f}')
else:
    print('【静止段噪声】数据太少')
print()

# yaw/pitch 跳变
print('【角度跳变】')
print(f'  yaw:   std={std(dya):.4f}  max_jump={max(dya):.4f}')
print(f'  pitch: std={std(dpi):.4f}  max_jump={max(dpi):.4f}')
print()

# 跳变分布
bins = [0.001, 0.005, 0.01, 0.02, 0.05, 0.1, 1.0]
print('【yaw 跳变分布】')
for i in range(len(bins)-1):
    cnt = sum(1 for d in dya if bins[i] <= d < bins[i+1])
    pct = cnt / len(dya) * 100
    bar = '#' * int(pct)
    print(f'  [{bins[i]:.3f}-{bins[i+1]:.3f}) {pct:5.1f}% {bar}')
cnt = sum(1 for d in dya if d >= bins[-1])
pct = cnt / len(dya) * 100
print(f'  [>{bins[-1]})           {pct:5.1f}%')
print()

# 诊断
print('='*50)
print('【诊断】')
if drop > 0.3:
    print('  丢帧严重(>30%)，检查 face_detect 是否稳定')
if fps < 20:
    print(f'  有效帧率低({fps:.0f}Hz)，相机或检测节点有延迟')
if len(static_idx) > 5:
    sz = std([rz[i] for i in static_idx])
    if sz > 0.1:
        print('  Z轴噪声大(>10cm)，深度相机噪声是主要问题')
if std(dya) > 0.03:
    print('  yaw 跳变过大，指向解算输入不稳定')
if std(dpi) > 0.02:
    print('  pitch 跳变过大，需要加强滤波或死区')

# KF 滤波效果对比
if len(static_idx) > 5:
    print()
    print('【KF 滤波效果（静止段）】')
    for raw_c, filt_c, name in [(rx,fx,'X(深度)'), (ry,fy,'Y'), (rz,fz,'Z')]:
        r_std = std([raw_c[i] for i in static_idx])
        f_std = std([filt_c[i] for i in static_idx])
        ratio = r_std / max(f_std, 1e-9)
        print(f'  {name}: raw_std={r_std:.4f}  filt_std={f_std:.4f}  降噪倍数={ratio:.0f}x')
