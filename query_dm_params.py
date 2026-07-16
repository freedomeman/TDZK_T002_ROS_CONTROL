#!/usr/bin/env python3
"""查询 DM 电机的 PMAX(21) / VMAX(22) / TMAX(23) 寄存器值

用法:
    python3 query_dm_params.py <motor_id> [can_interface]

示例 (查 neck_yaw, motor_id=3, can0):
    python3 query_dm_params.py 3

约束: 查询期间需要关闭 ros2_control (或先停掉 controller),
      避免 CAN 总线上其他帧干扰。
"""

import socket
import struct
import sys
import time


# CAN 帧格式: (can_id, can_dlc, data[8], flags)
CAN_MTU = 16
CANFD_MTU = 72
SOL_CAN_BASE = 100
CAN_RAW = 1


def pack_can_frame(can_id: int, data: bytes) -> bytes:
    """打包为标准 CAN 2.0 帧 (8 字节数据)"""
    assert len(data) == 8
    return struct.pack('=IB3x8s', can_id & 0x1FFFFFFF, 8, data)


def unpack_can_frame(frame: bytes):
    """解包 CAN 帧, 返回 (can_id, dlc, data)"""
    can_id, dlc, data = struct.unpack('=IB3x8s', frame[:16])
    return can_id & 0x1FFFFFFF, dlc, data[:dlc]


def send_param_query(sock, motor_id: int, register_id: int):
    """发送 DM 参数查询帧 (0x33 命令)"""
    data = bytes([
        motor_id & 0xFF,      # data[0]: motor_id 低字节
        motor_id >> 8,         # data[1]: motor_id 高字节
        0x33,                  # data[2]: 读参数命令
        register_id,           # data[3]: 寄存器编号
        0xFF, 0xFF, 0xFF, 0xFF # data[4..7]: 填充
    ])
    frame = pack_can_frame(0x7FF, data)
    sock.send(frame)
    print(f"  → 发送查询: motor_id={motor_id}, register={register_id} (0x{register_id:02X})")


def recv_response(sock, motor_id: int, register_id: int, timeout: float = 2.0):
    """等待并解析参数应答帧

    应答格式 (来自 motor_id+1):
      data[0..1]: motor_id (little-endian)
      data[2]: 0x33
      data[3]: register_id
      data[4..7]: float 值 (little-endian)
    """
    sock.settimeout(timeout)
    deadline = time.time() + timeout

    while time.time() < deadline:
        try:
            raw = sock.recv(64)
        except socket.timeout:
            break

        can_id, dlc, data = unpack_can_frame(raw)

        # 参数应答: 来自 motor_id+1, 8 字节, data[2]==0x33
        if can_id != motor_id + 1:
            continue
        if dlc != 8:
            continue
        if data[2] != 0x33:
            continue
        if data[3] != register_id:
            continue

        # 解析浮点值
        rsp_motor_id = data[0] | (data[1] << 8)
        value = struct.unpack('<f', data[4:8])[0]
        return rsp_motor_id, value

    return None, None


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    motor_id = int(sys.argv[1])
    can_if = sys.argv[2] if len(sys.argv) > 2 else 'can0'

    # 查询的寄存器
    registers = {
        21: 'PMAX (PosMax, rad)',
        22: 'VMAX (SpdMax, rad/s)',
        23: 'TMAX (TauMax, N·m)',
    }

    try:
        sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        sock.bind((can_if,))
    except OSError as e:
        print(f"❌ 无法绑定 {can_if}: {e}")
        print("   请确认 CAN 接口已启动: sudo ip link set {can_if} up type can bitrate 1000000")
        sys.exit(1)

    print(f"📡 查询 DM 电机 motor_id={motor_id} (应答 ID={motor_id + 1})  on {can_if}")
    print()

    results = {}
    for reg_id, desc in registers.items():
        send_param_query(sock, motor_id, reg_id)
        time.sleep(0.05)  # 给电机一点响应时间

        rsp_id, value = recv_response(sock, motor_id, reg_id)

        if rsp_id is not None:
            print(f"  ← 应答: reg={reg_id}  {desc}")
            print(f"     值 = {value:.6f}")
            results[reg_id] = value
        else:
            print(f"  ← 应答: reg={reg_id}  {desc}")
            print(f"     ⚠️ 超时，未收到应答")

        time.sleep(0.02)

    sock.close()

    # 打印汇总
    print()
    print("=" * 55)
    print("📊 查询结果汇总")
    print("=" * 55)
    for reg_id in [21, 22, 23]:
        desc = registers[reg_id]
        if reg_id in results:
            print(f"  {desc:35s} = {results[reg_id]:.4f}")
        else:
            print(f"  {desc:35s} = ??? (未查询到)")
    print("=" * 55)
    print()
    print("将这 3 个值填入 dm_motor_driver.cpp 的 limit_param[2]:")
    if all(r in results for r in [21, 22, 23]):
        pmax = results[21]
        vmax = results[22]
        tmax = results[23]
        print(f"  {{{pmax:.1f}, {vmax:.1f}, {tmax:.1f}, 500, 5}},   // DM_G6220")
    else:
        print("  (部分值查询失败，请手动填写)")
    print()


if __name__ == '__main__':
    main()
