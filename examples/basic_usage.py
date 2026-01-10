"""
F-IMOP Basic Usage Example
==========================

This script demonstrates the minimal setup required to use F-IMOP
as a safety filter for trajectory tracking control.

运行方式:
    python examples/basic_usage.py
"""

import numpy as np
import sys
import os

# 添加包路径（开发模式）
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from fimop import FIMOPController
from fimop.dynamics import TwoLinkArm
from fimop.utils import generate_figure8_trajectory


def simple_pd_controller(q, dq, q_d, dq_d, Kp=100.0, Kd=20.0):
    """简单的 PD 控制器作为名义控制器。"""
    e = q_d - q
    de = dq_d - dq
    return Kp * e + Kd * de


def run_basic_demo():
    """运行基础 F-IMOP 演示。"""
    print("=" * 60)
    print("F-IMOP Basic Usage Demo")
    print("=" * 60)
    
    # ========================================
    # Step 1: 创建动力学模型
    # ========================================
    robot = TwoLinkArm(m1=1.0, m2=1.0, l1=1.0, l2=1.0)
    print(f"\n[1] 创建机器人模型: {robot}")
    
    # ========================================
    # Step 2: 初始化 F-IMOP 控制器
    # ========================================
    fimop = FIMOPController(
        dynamics_model=robot,
        Lambda=5.0,        # 滤波误差增益
        decay_rate=2.0     # 能量衰减率
    )
    print(f"[2] 初始化 F-IMOP: {fimop}")
    
    # ========================================
    # Step 3: 设置仿真参数
    # ========================================
    dt = 0.002           # 2ms 控制周期 (500 Hz)
    duration = 5.0       # 仿真时长
    steps = int(duration / dt)
    
    # 轨迹生成器
    get_ref = generate_figure8_trajectory(dof=2, frequency=0.5*np.pi, amplitude=0.5)
    
    # 初始状态
    q = np.zeros(2)
    dq = np.zeros(2)
    
    # 数据记录
    errors = []
    corrections = []
    
    print(f"[3] 仿真设置: dt={dt}s, duration={duration}s, steps={steps}")
    
    # ========================================
    # Step 4: 控制循环
    # ========================================
    print("\n[4] 开始仿真...")
    
    for i in range(steps):
        t = i * dt
        
        # 获取参考轨迹
        q_d, dq_d, ddq_d = get_ref(t)
        
        # 名义控制器（简单 PD）
        tau_nom = simple_pd_controller(q, dq, q_d, dq_d)
        
        # F-IMOP 安全过滤
        tau_safe, info = fimop.compute_safe_control(
            q, dq, q_d, dq_d, ddq_d, tau_nom
        )
        
        # 仿真动力学（正向积分）
        ddq = robot.forward_dynamics(q, dq, tau_safe)
        dq = dq + ddq * dt
        q = q + dq * dt
        
        # 记录数据
        errors.append(np.linalg.norm(q - q_d))
        corrections.append(info['correction'])
    
    # ========================================
    # Step 5: 结果统计
    # ========================================
    errors = np.array(errors)
    corrections = np.array(corrections)
    state = fimop.get_filter_state()
    
    print("\n" + "=" * 60)
    print("仿真结果")
    print("=" * 60)
    print(f"  平均跟踪误差: {errors.mean():.6f} rad")
    print(f"  最大跟踪误差: {errors.max():.6f} rad")
    print(f"  RMSE:         {np.sqrt((errors**2).mean()):.6f} rad")
    print(f"  F-IMOP 激活比例: {state['activation_ratio']*100:.1f}%")
    print(f"  平均修正量:   {corrections.mean():.4f} Nm")
    print("=" * 60)
    
    return errors, corrections


if __name__ == "__main__":
    run_basic_demo()
