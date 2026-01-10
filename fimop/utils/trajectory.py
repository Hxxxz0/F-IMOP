"""
Trajectory generation utilities for F-IMOP experiments.

These functions generate smooth reference trajectories for testing and demos.
"""

import numpy as np
from typing import Tuple, Callable


def generate_figure8_trajectory(dof: int, 
                                 frequency: float = 0.5 * np.pi,
                                 amplitude: float = 0.5) -> Callable:
    """
    生成"8"字形周期轨迹生成器。
    
    返回一个函数，给定时间 t 返回 (q_d, dq_d, ddq_d)。
    
    Args:
        dof: 自由度数量
        frequency: 角频率 (rad/s)
        amplitude: 振幅 (rad)
    
    Returns:
        get_ref: 轨迹生成函数 get_ref(t) -> (q_d, dq_d, ddq_d)
    
    Example:
        >>> get_ref = generate_figure8_trajectory(dof=2)
        >>> q_d, dq_d, ddq_d = get_ref(t=1.0)
    """
    def get_ref(t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        w = frequency
        A = amplitude
        
        if dof == 2:
            # 2-DOF: 经典"8"字形
            q_d = np.array([A * np.sin(w * t), A * np.cos(w * t)])
            dq_d = np.array([A * w * np.cos(w * t), -A * w * np.sin(w * t)])
            ddq_d = np.array([-A * w**2 * np.sin(w * t), -A * w**2 * np.cos(w * t)])
        else:
            # 高维: 交替 sin/cos
            s, c = np.sin(w * t), np.cos(w * t)
            amplitudes = np.linspace(A, A * 0.4, dof)  # 远端振幅递减
            
            q_d = np.zeros(dof)
            dq_d = np.zeros(dof)
            ddq_d = np.zeros(dof)
            
            for i in range(dof):
                if i % 2 == 0:
                    q_d[i] = amplitudes[i] * s
                    dq_d[i] = amplitudes[i] * w * c
                    ddq_d[i] = -amplitudes[i] * w**2 * s
                else:
                    q_d[i] = amplitudes[i] * c
                    dq_d[i] = -amplitudes[i] * w * s
                    ddq_d[i] = -amplitudes[i] * w**2 * c
        
        return q_d, dq_d, ddq_d
    
    return get_ref


def generate_step_trajectory(dof: int,
                              step_time: float = 1.0,
                              target: np.ndarray = None) -> Callable:
    """
    生成阶跃轨迹（用于静态定位测试）。
    
    Args:
        dof: 自由度数量
        step_time: 阶跃发生时间
        target: 目标位置，默认为零
    
    Returns:
        get_ref: 轨迹生成函数
    """
    if target is None:
        target = np.zeros(dof)
    
    def get_ref(t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        if t < step_time:
            return np.zeros(dof), np.zeros(dof), np.zeros(dof)
        else:
            return target.copy(), np.zeros(dof), np.zeros(dof)
    
    return get_ref
