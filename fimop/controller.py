"""
FIMOPController: Core implementation of the F-IMOP safety filter.

This module implements the closed-form projection algorithm described in the paper:
"F-IMOP: Filtered-Error Inertia-Metric Optimal Projection for Safe Robot Control"
"""

import numpy as np
from typing import Union, Tuple, Dict, Any, Optional

from .dynamics.base import DynamicsModel


class FIMOPController:
    """
    F-IMOP 安全控制器核心实现。
    
    该类实现了基于惯性度量的最小扰动投影算法。它接收名义控制器的输出，
    在必要时进行修正以保证系统满足指数稳定约束。
    
    核心算法（论文公式 14-16）：
    1. 计算滤波误差 s = de + Λe
    2. 检查约束 s^T τ_nom ≤ s^T τ_ref - (λ/2) s^T M s
    3. 若违反，计算闭式投影：τ* = τ_nom - μ·M·s，其中 μ = (s^T τ_nom - b) / (s^T M s)
    
    Attributes:
        model: 动力学模型实例
        Lambda: 滤波误差增益矩阵
        decay_rate: 能量衰减率 λ
        slack_tolerance: 数值容差
    
    Example:
        >>> from fimop import FIMOPController
        >>> from fimop.dynamics import TwoLinkArm
        >>> controller = FIMOPController(TwoLinkArm(), Lambda=5.0, decay_rate=2.0)
        >>> tau_safe, info = controller.compute_safe_control(q, dq, q_d, dq_d, ddq_d, tau_nom)
    """
    
    def __init__(self, 
                 dynamics_model: DynamicsModel,
                 Lambda: Union[float, np.ndarray] = 5.0,
                 decay_rate: float = 2.0,
                 slack_tolerance: float = 1e-8):
        """
        初始化安全控制器。
        
        Args:
            dynamics_model: 实现了 DynamicsModel 接口的动力学模型对象
            Lambda: 滤波误差增益 Λ (s = de + Λe)。
                    可以是标量（对所有关节使用相同值）或 (n,n) 对角矩阵。
            decay_rate: 能量衰减率 λ，控制 V_dot ≤ -λV 的收敛速度。
                        较大的值意味着更激进的修正。
            slack_tolerance: 数值计算容差，用于判断约束是否被违反。
                            设置适当的值可以避免由于浮点误差导致的抖动。
        """
        self.model = dynamics_model
        self.decay_rate = decay_rate
        self.slack_tolerance = slack_tolerance
        
        # 将标量 Lambda 转换为对角矩阵
        self.Lambda = self._ensure_matrix(Lambda, self.model.dof)
        
        # 内部状态记录（用于调试）
        self._last_V = 0.0
        self._last_V_dot = 0.0
        self._activation_count = 0
        self._total_calls = 0
    
    def _ensure_matrix(self, value: Union[float, np.ndarray], n: int) -> np.ndarray:
        """将标量或向量转换为 (n, n) 对角矩阵。"""
        if np.isscalar(value):
            return np.eye(n) * value
        elif isinstance(value, np.ndarray):
            if value.ndim == 1:
                return np.diag(value)
            elif value.ndim == 2 and value.shape == (n, n):
                return value
        raise ValueError(f"Lambda must be scalar, 1D array of length {n}, or ({n}, {n}) matrix")
    
    def compute_safe_control(self,
                             q: np.ndarray,
                             dq: np.ndarray,
                             q_target: np.ndarray,
                             dq_target: np.ndarray,
                             ddq_target: np.ndarray,
                             tau_nominal: np.ndarray) -> Tuple[np.ndarray, Dict[str, Any]]:
        """
        计算满足安全约束的控制力矩（核心方法）。
        
        该方法实现了论文中的闭式解投影算法：
        - 若名义力矩已满足稳定约束，直接返回不做修改
        - 若违反约束，计算惯性度量下的最小修正
        
        Args:
            q: 当前关节位置 [n_dof]
            dq: 当前关节速度 [n_dof]  
            q_target: 期望位置 q_d [n_dof]
            dq_target: 期望速度 dq_d [n_dof]
            ddq_target: 期望加速度 ddq_d [n_dof]
            tau_nominal: 名义控制器输出的力矩 [n_dof]
        
        Returns:
            tau_safe: 修正后的安全力矩 [n_dof]
            info: 调试信息字典，包含:
                - 'active' (bool): 安全约束是否被激活（True 表示进行了修正）
                - 'correction' (float): 力矩修正量的范数 ||τ* - τ_nom||
                - 'V' (float): 当前 Lyapunov 能量值 V = 0.5 * s^T M s
                - 'violation' (float): 约束违反程度（正值表示需要修正）
                - 's_norm' (float): 滤波误差范数 ||s||
        """
        self._total_calls += 1
        
        # 确保输入为 numpy 数组
        q = np.asarray(q, dtype=np.float64)
        dq = np.asarray(dq, dtype=np.float64)
        q_target = np.asarray(q_target, dtype=np.float64)
        dq_target = np.asarray(dq_target, dtype=np.float64)
        ddq_target = np.asarray(ddq_target, dtype=np.float64)
        tau_nominal = np.asarray(tau_nominal, dtype=np.float64)
        
        # ============================================
        # Step 1: 计算跟踪误差和滤波误差
        # ============================================
        # e = q - q_d (位置误差，注意符号定义)
        e = q - q_target
        de = dq - dq_target
        
        # 滤波误差: s = de + Λ·e (论文公式 3)
        s = de + self.Lambda @ e
        
        # ============================================
        # Step 2: 获取动力学矩阵
        # ============================================
        M, C, G = self.model.get_dynamics(q, dq)
        
        # ============================================
        # Step 3: 计算参考动力学力矩 τ_ref (论文公式 6)
        # ============================================
        # 参考速度: dq_r = dq_d - Λ·e
        # 参考加速度: ddq_r = ddq_d - Λ·de
        dq_r = dq_target - self.Lambda @ e
        ddq_r = ddq_target - self.Lambda @ de
        
        # τ_ref = M·ddq_r + C·dq_r + G
        tau_ref = M @ ddq_r + C @ dq_r + G
        
        # ============================================
        # Step 4: 计算 Lyapunov 函数和约束边界
        # ============================================
        # V = 0.5 * s^T M s (论文公式 7)
        Ms = M @ s
        s_M_s = s @ Ms
        V = 0.5 * s_M_s
        
        # 约束边界: b = s^T τ_ref - (λ/2) s^T M s (论文公式 12)
        b = s @ tau_ref - 0.5 * self.decay_rate * s_M_s
        
        # ============================================
        # Step 5: 检查约束并计算投影
        # ============================================
        # 约束: s^T τ ≤ b
        lhs = s @ tau_nominal
        violation = lhs - b
        
        # 构建返回信息
        info = {
            'active': False,
            'correction': 0.0,
            'V': V,
            'violation': violation,
            's_norm': np.linalg.norm(s),
            'e_norm': np.linalg.norm(e),
        }
        
        # 约束已满足，直接返回名义力矩
        if violation <= self.slack_tolerance:
            self._last_V = V
            return tau_nominal.copy(), info
        
        # ============================================
        # Step 6: 计算闭式投影 (论文公式 16)
        # ============================================
        # μ = (s^T τ_nom - b) / (s^T M s)
        # τ* = τ_nom - μ·M·s
        
        # 避免除以零（当 s ≈ 0 时，V ≈ 0，约束自然满足）
        if s_M_s < 1e-12:
            self._last_V = V
            return tau_nominal.copy(), info
        
        mu = violation / s_M_s
        tau_safe = tau_nominal - mu * Ms
        
        # 更新统计信息
        self._activation_count += 1
        self._last_V = V
        
        # 计算实际的 V_dot 用于验证
        # V_dot = s^T (τ - τ_ref) (由于斜对称性，非线性项消除)
        self._last_V_dot = s @ (tau_safe - tau_ref)
        
        info['active'] = True
        info['correction'] = np.linalg.norm(tau_safe - tau_nominal)
        info['mu'] = mu
        
        return tau_safe, info
    
    def get_filter_state(self) -> Dict[str, Any]:
        """
        获取控制器的内部状态（用于日志和调试）。
        
        Returns:
            状态字典，包含:
                - 'total_calls': 总调用次数
                - 'activation_count': 约束激活次数
                - 'activation_ratio': 激活比例
                - 'last_V': 最近一次的 Lyapunov 值
                - 'Lambda': 滤波增益矩阵
                - 'decay_rate': 衰减率
        """
        return {
            'total_calls': self._total_calls,
            'activation_count': self._activation_count,
            'activation_ratio': self._activation_count / max(1, self._total_calls),
            'last_V': self._last_V,
            'Lambda': self.Lambda.copy(),
            'decay_rate': self.decay_rate,
        }
    
    def reset_statistics(self):
        """重置内部统计计数器。"""
        self._activation_count = 0
        self._total_calls = 0
        self._last_V = 0.0
        self._last_V_dot = 0.0
    
    def __repr__(self) -> str:
        return (f"FIMOPController(dof={self.model.dof}, "
                f"decay_rate={self.decay_rate}, "
                f"Lambda=diag{np.diag(self.Lambda).tolist()})")
