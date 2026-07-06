from __future__ import annotations

import importlib.util
import math
from pathlib import Path
from typing import Any, Dict

import numpy as np


SOURCE_SOLVER = Path(__file__).parent / "parallelJoint_g1.py"
RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0

_SOURCE_MODULE = None


def load_source_module(source_path: Path = SOURCE_SOLVER):
    global _SOURCE_MODULE
    if _SOURCE_MODULE is not None:
        return _SOURCE_MODULE

    if not source_path.exists():
        raise FileNotFoundError(f"source solver not found: {source_path}")

    spec = importlib.util.spec_from_file_location("source_parallel_joint", source_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot load source solver from {source_path}")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    _SOURCE_MODULE = module
    return module


def apply_head_parameters(model):
    model.h0 = np.array([0.0, 0.0, 0.0])

    model.ra01 = np.array([-0.031, 0.0255, 0.05145])
    model.ra02 = np.array([-0.031, -0.0255, 0.05145])

    model.rb01 = np.array([-0.031, 0.0355, 0.05145])
    model.rb02 = np.array([-0.031, -0.0355, 0.05145])

    model.rc01 = np.array([-0.031, 0.0300, -0.02595])
    model.rc02 = np.array([-0.031, -0.0300, -0.02595])
    model.hc01 = model.rc01 - model.h0
    model.hc02 = model.rc02 - model.h0

    model.lbar1 = np.linalg.norm(model.rb01 - model.ra01)
    model.lbar2 = np.linalg.norm(model.rb02 - model.ra02)
    model.lrod1 = np.linalg.norm(model.rb01 - model.rc01)
    model.lrod2 = np.linalg.norm(model.rb02 - model.rc02)

    model.re01 = model.rc01 + np.array([0.08, 0.0, 0.0])
    model.re02 = model.rc02 + np.array([0.08, 0.0, 0.0])
    model.he01 = model.re01 - model.h0
    model.he02 = model.re02 - model.h0

    model.rab01_y = np.array([0.0, model.lbar1, 0.0])
    model.rab02_y = np.array([0.0, -model.lbar2, 0.0])

    model.m01 = 0.0
    model.m02 = 0.0
    return model


def create_model():
    module = load_source_module()
    return apply_head_parameters(module.parallelAnkle())


def rz(yaw: float) -> np.ndarray:
    return np.array(
        [
            [math.cos(yaw), -math.sin(yaw), 0.0],
            [math.sin(yaw), math.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )


def rotate_points(points: Dict[str, np.ndarray], yaw: float) -> Dict[str, np.ndarray]:
    yaw_rotation = rz(yaw)
    return {name: np.array(yaw_rotation @ point, dtype=float) for name, point in points.items()}


def mechanism_points(
    model: Any,
    pitch: float,
    roll: float,
    theta1: float,
    theta2: float,
    yaw: float = 0.0,
) -> Dict[str, np.ndarray]:
    rh = model.RyPlot(pitch) @ model.h0

    rb1 = model.ra01 + model.RxPlot(theta1 + model.m01) @ model.rab01_y
    rc1 = rh + model.xrot(pitch, roll) @ model.hc01
    re1 = rh + model.xrot(pitch, roll) @ model.he01

    rb2 = model.ra02 + model.RxPlot(theta2 + model.m02) @ model.rab02_y
    rc2 = rh + model.xrot(pitch, roll) @ model.hc02
    re2 = rh + model.xrot(pitch, roll) @ model.he02

    points = {
        "origin": np.array([0.0, 0.0, 0.0]),
        "H": np.array(rh, dtype=float),
        "A1": np.array(model.ra01, dtype=float),
        "B1": np.array(rb1, dtype=float),
        "C1": np.array(rc1, dtype=float),
        "E1": np.array(re1, dtype=float),
        "A2": np.array(model.ra02, dtype=float),
        "B2": np.array(rb2, dtype=float),
        "C2": np.array(rc2, dtype=float),
        "E2": np.array(re2, dtype=float),
    }
    return rotate_points(points, yaw)


def constraint_report(model: Any, points: Dict[str, np.ndarray]) -> Dict[str, float]:
    return {
        "bar1_len": float(np.linalg.norm(points["A1"] - points["B1"])),
        "bar1_target": float(model.lbar1),
        "rod1_len": float(np.linalg.norm(points["B1"] - points["C1"])),
        "rod1_target": float(model.lrod1),
        "bar2_len": float(np.linalg.norm(points["A2"] - points["B2"])),
        "bar2_target": float(model.lbar2),
        "rod2_len": float(np.linalg.norm(points["B2"] - points["C2"])),
        "rod2_target": float(model.lrod2),
    }


def source_polyline_order() -> list[str]:
    return ["origin", "C1", "B1", "A1", "E1", "C2", "B2", "A2", "E2", "C2", "C1", "E1", "E2"]


def compute_state_from_pose(model: Any, pitch: float, roll: float, yaw: float = 0.0) -> Dict[str, Any]:
    theta1, theta2, error_state = model.ik(pitch, roll)
    points = mechanism_points(model, pitch, roll, theta1, theta2, yaw)
    return {
        "pitch": float(pitch),
        "roll": float(roll),
        "yaw": float(yaw),
        "theta1": float(theta1),
        "theta2": float(theta2),
        "error_state": int(error_state),
        "points": points,
        "constraints": constraint_report(model, points),
    }


def compute_state_from_motors(
    model: Any,
    theta1: float,
    theta2: float,
    initial_pitch: float = 0.0,
    initial_roll: float = 0.0,
    yaw: float = 0.0,
) -> Dict[str, Any]:
    pitch, roll, error_state = model.fw(initial_pitch, initial_roll, theta1, theta2)
    points = mechanism_points(model, pitch, roll, theta1, theta2, yaw)
    return {
        "pitch": float(pitch),
        "roll": float(roll),
        "yaw": float(yaw),
        "theta1": float(theta1),
        "theta2": float(theta2),
        "error_state": int(error_state),
        "points": points,
        "constraints": constraint_report(model, points),
    }


def is_valid_state(state: Dict[str, Any]) -> bool:
    return state["error_state"] == 0


# ==================== 力矩控制方案 ====================
from dataclasses import dataclass


@dataclass(frozen=True)
class HeadControlGains:
    """姿态空间 PD 增益与限幅"""
    kp_pitch: float
    kd_pitch: float
    kp_roll: float
    kd_roll: float
    max_joint_torque: float | None = None
    max_motor_torque: float | None = None


def _limit_vector(values: np.ndarray, limit: float | None) -> np.ndarray:
    if limit is None:
        return values
    bound = abs(float(limit))
    return np.clip(values, -bound, bound)


def compute_pose_velocity_from_motors(
    model, theta1, theta2, theta1_dot, theta2_dot,
    initial_pitch=0.0, initial_roll=0.0, yaw=0.0,
    singular_condition_limit=None, use_pinv=False,
):
    """电机角/速度 → 实际姿态 + 姿态速度 (经由 Jacobian)"""
    state = compute_state_from_motors(model, theta1, theta2, initial_pitch, initial_roll, yaw)
    base = {
        **state,
        "pitch_dot": 0.0, "roll_dot": 0.0,
        "theta1_dot": float(theta1_dot), "theta2_dot": float(theta2_dot),
        "jacobian": None, "jacobian_condition": float('inf'),
        "fk_error_state": 0, "jac_error_state": 0,
    }
    if state["error_state"] != 0:
        base["fk_error_state"] = int(state["error_state"])
        base["error_state"] = int(state["error_state"])
        return base

    jacobian, jac_error_state = model.Jac(state["pitch"], state["roll"], theta1, theta2)
    jac_cond = float(np.linalg.cond(jacobian))
    if jac_error_state != 0 or \
       (singular_condition_limit is not None and jac_cond > singular_condition_limit):
        base["jacobian"] = jacobian
        base["jacobian_condition"] = jac_cond
        base["jac_error_state"] = int(jac_error_state)
        base["error_state"] = 2
        return base

    theta_dot = np.array([theta1_dot, theta2_dot], dtype=float)
    try:
        roll_pitch_dot = (np.linalg.pinv(jacobian) @ theta_dot if use_pinv
                          else np.linalg.solve(jacobian, theta_dot))
    except np.linalg.LinAlgError:
        base["jacobian"] = jacobian
        base["jacobian_condition"] = jac_cond
        base["error_state"] = 3
        return base

    return {
        **state,
        "pitch_dot": float(roll_pitch_dot[1]),
        "roll_dot":  float(roll_pitch_dot[0]),
        "theta1_dot": float(theta1_dot),
        "theta2_dot": float(theta2_dot),
        "jacobian": jacobian,
        "jacobian_condition": jac_cond,
        "error_state": 0,
    }


def compute_motor_torque_command(
    model, target_pitch, target_roll,
    theta1, theta2, theta1_dot, theta2_dot,
    gains: HeadControlGains,
    target_pitch_dot=0.0, target_roll_dot=0.0,
    initial_pitch=0.0, initial_roll=0.0, yaw=0.0,
    singular_condition_limit=None, use_pinv=False,
):
    """目标姿态 + 电机状态 → 电机力矩命令 (PD + Jacobian 反解)"""
    vel = compute_pose_velocity_from_motors(
        model, theta1, theta2, theta1_dot, theta2_dot,
        initial_pitch, initial_roll, yaw,
        singular_condition_limit, use_pinv,
    )

    target_theta1, target_theta2, target_ik_err = model.ik(target_pitch, target_roll)

    base = {
        **vel,
        "target_pitch": float(target_pitch),
        "target_roll": float(target_roll),
        "target_pitch_dot": float(target_pitch_dot),
        "target_roll_dot": float(target_roll_dot),
        "target_theta1": float(target_theta1),
        "target_theta2": float(target_theta2),
        "target_error_state": int(target_ik_err),
        "pitch_error": float(target_pitch - vel["pitch"]),
        "roll_error": float(target_roll - vel["roll"]),
        "pitch_dot_error": float(target_pitch_dot - vel["pitch_dot"]),
        "roll_dot_error": float(target_roll_dot - vel["roll_dot"]),
        "tau_pitch": 0.0, "tau_roll": 0.0,
        "theta1_torque": 0.0, "theta2_torque": 0.0,
    }
    if vel["error_state"] != 0:
        return base
    if target_ik_err != 0:
        base["error_state"] = 5
        return base

    tau_pitch = gains.kp_pitch * base["pitch_error"] + gains.kd_pitch * base["pitch_dot_error"]
    tau_roll  = gains.kp_roll  * base["roll_error"]  + gains.kd_roll  * base["roll_dot_error"]
    joint_tau = _limit_vector(np.array([tau_roll, tau_pitch]), gains.max_joint_torque)

    try:
        motor_tau = (np.linalg.pinv(vel["jacobian"].T) @ joint_tau if use_pinv
                     else np.linalg.solve(vel["jacobian"].T, joint_tau))
    except np.linalg.LinAlgError:
        base["tau_roll"] = float(joint_tau[0])
        base["tau_pitch"] = float(joint_tau[1])
        base["error_state"] = 4
        return base

    motor_tau = _limit_vector(motor_tau, gains.max_motor_torque)
    base["tau_roll"]  = float(joint_tau[0])
    base["tau_pitch"] = float(joint_tau[1])
    base["theta1_torque"] = float(motor_tau[0])
    base["theta2_torque"] = float(motor_tau[1])
    base["error_state"] = 0
    return base
