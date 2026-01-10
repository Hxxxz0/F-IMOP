"""
F-IMOP Paper Experiments
========================

This script reproduces all experiments from the F-IMOP paper:
1. Baseline Tracking: Normal trajectory following
2. Disturbance Rejection: External impact response
3. Model Mismatch: Robustness to parameter errors

Results are saved to the results/ directory.

运行方式:
    python examples/paper_experiments.py
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from typing import Dict, List, Tuple

# 添加包路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from fimop import FIMOPController
from fimop.dynamics import TwoLinkArm, SixLinkArm, DynamicsModel
from fimop.utils import generate_figure8_trajectory


# ==========================================
# 控制器实现
# ==========================================

def get_pid_torque(q, dq, q_d, dq_d, int_e, Kp, Kd, Ki):
    """PID 控制器。"""
    e = q_d - q
    de = dq_d - dq
    return Kp @ e + Kd @ de + Ki @ int_e


def get_ctc_torque(q, dq, q_d, dq_d, ddq_d, model: DynamicsModel, Kp, Kd):
    """计算力矩控制 (CTC) - 理想控制器。"""
    M, C, G = model.get_dynamics(q, dq)
    e = q_d - q
    de = dq_d - dq
    aq = ddq_d + Kp @ e + Kd @ de
    return M @ aq + C @ dq + G


# ==========================================
# 实验引擎
# ==========================================

def run_experiment(
    arm_type: str,
    controller_type: str,
    scenario: str,
    duration: float = 5.0,
    mismatch_ratio: float = 1.0
) -> Dict:
    """
    运行单个实验配置。
    
    Args:
        arm_type: "2DOF" 或 "6DOF"
        controller_type: "PID", "CTC", 或 "F-IMOP"
        scenario: "Baseline", "Disturbance", 或 "Mismatch"
        duration: 仿真时长
        mismatch_ratio: 模型失配比例（控制器认为的质量/真实质量）
    
    Returns:
        包含 time, err, tau, mod, V 的结果字典
    """
    dt = 0.002
    steps = int(duration / dt)
    time_span = np.linspace(0, duration, steps)
    
    # 创建真实机器人（用于仿真）
    if arm_type == "2DOF":
        real_robot = TwoLinkArm(m_scale=1.0)
        dof = 2
        Kp = np.eye(dof) * 100.0
        Kd = np.eye(dof) * 20.0
        Ki = np.eye(dof) * 50.0
        Lambda = np.eye(dof) * 5.0
    else:
        real_robot = SixLinkArm(m_scale=1.0)
        dof = 6
        Kp = np.eye(dof) * 80.0
        Kd = np.eye(dof) * 15.0
        Ki = np.eye(dof) * 40.0
        Lambda = np.eye(dof) * 4.0
    
    # 创建控制器使用的模型（可能有失配）
    if arm_type == "2DOF":
        ctrl_model = TwoLinkArm(m_scale=mismatch_ratio)
    else:
        ctrl_model = SixLinkArm(m_scale=mismatch_ratio)
    
    # 创建 F-IMOP 控制器
    fimop = FIMOPController(ctrl_model, Lambda=Lambda, decay_rate=5.0)
    
    # 状态初始化
    q = np.zeros(dof)
    dq = np.zeros(dof)
    int_e = np.zeros(dof)
    
    # 轨迹生成
    def get_ref(t):
        if scenario in ["Baseline", "Mismatch"]:
            return generate_figure8_trajectory(dof)(t)
        else:  # Disturbance - 保持静止
            return np.zeros(dof), np.zeros(dof), np.zeros(dof)
    
    # 数据记录
    log = {"time": [], "err": [], "tau": [], "mod": [], "V": []}
    
    for i, t in enumerate(time_span):
        q_d, dq_d, ddq_d = get_ref(t)
        
        # 外部扰动（仅 Disturbance 场景）
        ext_tau = np.zeros(dof)
        if scenario == "Disturbance" and 1.0 <= t <= 1.2:
            ext_tau[0] = 50.0
            if dof > 2:
                ext_tau[3] = 30.0
        
        # 计算控制力矩
        if controller_type == "PID":
            tau_out = get_pid_torque(q, dq, q_d, dq_d, int_e, Kp, Kd, Ki)
            tau_nom = tau_out
            
        elif controller_type == "CTC":
            tau_out = get_ctc_torque(q, dq, q_d, dq_d, ddq_d, ctrl_model, Kp * 2, Kd * 2)
            tau_nom = tau_out
            
        elif controller_type == "F-IMOP":
            # 名义控制器: PD + 重力补偿
            tau_pd = get_pid_torque(q, dq, q_d, dq_d, int_e, Kp, Kd, np.zeros((dof, dof)))
            _, _, G_est = ctrl_model.get_dynamics(q, dq)
            tau_nom = tau_pd + G_est
            
            # F-IMOP 过滤
            tau_out, info = fimop.compute_safe_control(q, dq, q_d, dq_d, ddq_d, tau_nom)
        
        # 正向动力学仿真
        ddq = real_robot.forward_dynamics(q, dq, tau_out + ext_tau)
        dq = dq + ddq * dt
        q = q + dq * dt
        
        # 更新积分误差
        int_e = np.clip(int_e + (q_d - q) * dt, -5.0, 5.0)
        
        # 记录
        log["time"].append(t)
        log["err"].append(np.linalg.norm(q_d - q))
        log["tau"].append(np.linalg.norm(tau_out))
        log["mod"].append(np.linalg.norm(tau_out - tau_nom) if controller_type == "F-IMOP" else 0.0)
        
        # 计算 V
        e = q - q_d
        de = dq - dq_d
        s = de + Lambda @ e
        M, _, _ = real_robot.get_dynamics(q, dq)
        log["V"].append(0.5 * s @ (M @ s))
    
    return log


# ==========================================
# 绘图与报告
# ==========================================

def run_all_experiments():
    """运行所有实验并生成图表。"""
    # 创建结果目录
    results_dir = os.path.join(os.path.dirname(__file__), '..', 'results')
    os.makedirs(results_dir, exist_ok=True)
    
    scenarios = ["Baseline", "Disturbance", "Mismatch"]
    arms = ["2DOF", "6DOF"]
    
    print("=" * 60)
    print("F-IMOP Paper Experiments")
    print("=" * 60)
    
    for arm in arms:
        for scen in scenarios:
            print(f"\n运行实验: {arm} - {scen}...")
            
            # 设置失配比例
            ratio = 0.7 if scen == "Mismatch" else 1.0
            
            # 运行三个控制器
            res_pid = run_experiment(arm, "PID", scen, mismatch_ratio=ratio)
            res_fimop = run_experiment(arm, "F-IMOP", scen, mismatch_ratio=ratio)
            res_ctc = run_experiment(arm, "CTC", scen, mismatch_ratio=ratio)
            
            t = res_pid["time"]
            
            # 绘图
            plt.figure(figsize=(10, 8))
            
            # 子图 1: 跟踪误差
            plt.subplot(2, 1, 1)
            plt.title(f"{arm} Arm - {scen} Scenario", fontsize=14)
            plt.plot(t, res_pid["err"], 'r-', alpha=0.5, label="PID (Baseline)")
            plt.plot(t, res_ctc["err"], 'g--', alpha=0.6, label="CTC (Ideal)")
            plt.plot(t, res_fimop["err"], 'b-', linewidth=2.0, label="F-IMOP (Proposed)")
            plt.ylabel("Tracking Error Norm (rad)")
            plt.grid(True, alpha=0.3)
            plt.legend()
            
            # 子图 2: 力矩/修正量
            plt.subplot(2, 1, 2)
            if scen == "Baseline":
                plt.plot(t, res_fimop["mod"], 'b-', label="|τ* - τ_nom|")
                plt.fill_between(t, 0, res_fimop["mod"], color='blue', alpha=0.1)
                plt.ylabel("F-IMOP Intervention (Nm)")
            else:
                plt.plot(t, res_pid["tau"], 'r-', alpha=0.3, label="PID Torque")
                plt.plot(t, res_fimop["tau"], 'b-', linewidth=1.5, label="F-IMOP Torque")
                plt.ylabel("Total Control Effort (Nm)")
            
            plt.xlabel("Time (s)")
            plt.grid(True, alpha=0.3)
            plt.legend()
            
            # 保存
            filename = os.path.join(results_dir, f"Fig_{arm}_{scen}.png")
            plt.tight_layout()
            plt.savefig(filename, dpi=150)
            plt.close()
            print(f"  保存图表: {filename}")
    
    print("\n" + "=" * 60)
    print("所有实验完成！结果保存在 results/ 目录")
    print("=" * 60)


if __name__ == "__main__":
    run_all_experiments()
