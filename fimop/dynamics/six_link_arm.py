"""
SixLinkArm: 6-DOF robotic arm with decoupled (simplified) dynamics.

This model is used to demonstrate scalability of F-IMOP to higher-DOF systems.
It uses a diagonal inertia matrix for simplicity, but the interface remains
consistent with fully-coupled models.
"""

import numpy as np
from typing import Tuple
from .base import DynamicsModel


class SixLinkArm(DynamicsModel):
    """
    6-DOF 机械臂简化动力学模型。
    
    该模型使用解耦的对角惯性矩阵，主要用于验证算法在高维系统上的可扩展性。
    虽然是简化模型，但接口与完整动力学模型一致，可以无缝替换。
    
    这种解耦模型常见于工业机器人的近似控制，因为：
    1. 计算效率高
    2. 在低速运动时近似精度足够
    3. F-IMOP 的鲁棒性可以补偿模型误差
    
    Example:
        >>> arm = SixLinkArm()
        >>> M, C, G = arm.get_dynamics(q=np.zeros(6), dq=np.zeros(6))
        >>> print(M.shape)  # (6, 6)
    """
    
    def __init__(self,
                 masses: np.ndarray = None,
                 gravity_loads: np.ndarray = None,
                 m_scale: float = 1.0):
        """
        初始化 6-DOF 机械臂模型。
        
        Args:
            masses: 各关节的等效惯量，shape (6,)。
                    默认值模拟典型工业臂的惯量分布（近端大、远端小）。
            gravity_loads: 各关节的最大重力负载，shape (6,)。
                           实际重力 = gravity_loads * cos(q)。
            m_scale: 质量/惯量缩放因子，用于模型失配实验。
        """
        super().__init__(dof=6)
        
        # 默认惯量分布：近端关节惯量大，远端关节惯量小
        if masses is None:
            self.masses = np.array([1.0, 1.0, 1.0, 0.5, 0.5, 0.5]) * m_scale
        else:
            self.masses = np.asarray(masses) * m_scale
        
        # 默认重力负载分布
        if gravity_loads is None:
            self.gravity_loads = np.array([10.0, 8.0, 6.0, 3.0, 2.0, 1.0]) * m_scale
        else:
            self.gravity_loads = np.asarray(gravity_loads) * m_scale
    
    def get_dynamics(self, q: np.ndarray, dq: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        计算简化的 6-DOF 动力学矩阵。
        
        使用解耦模型：
        - M: 对角矩阵（各关节独立）
        - C: 零矩阵（忽略耦合效应）
        - G: 简化的正弦重力模型
        
        Args:
            q: 关节角度，shape (6,)
            dq: 关节角速度，shape (6,)
        
        Returns:
            M: 6x6 对角惯性矩阵
            C: 6x6 零矩阵
            G: 长度为 6 的重力向量
        """
        # 对角惯性矩阵
        M = np.diag(self.masses)
        
        # 简化：忽略科氏力/离心力（解耦近似）
        C = np.zeros((6, 6))
        
        # 简化重力模型：G_i = gravity_load_i * cos(q_i)
        G = self.gravity_loads * np.cos(q)
        
        return M, C, G
