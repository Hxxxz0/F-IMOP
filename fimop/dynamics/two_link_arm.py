"""
TwoLinkArm: 2-DOF planar robotic arm dynamics with full coupling.

This is a classic example for testing nonlinear control algorithms due to
its significant dynamic coupling between joints.
"""

import numpy as np
from typing import Tuple
from .base import DynamicsModel


class TwoLinkArm(DynamicsModel):
    """
    2-DOF 平面机械臂完整动力学模型。
    
    该模型包含完整的科氏力、离心力和重力项，是验证非线性控制算法的经典测试平台。
    两个连杆在垂直平面内运动，具有显著的动力学耦合。
    
    动力学方程: M(q)q̈ + C(q,q̇)q̇ + g(q) = τ
    
    参数可以通过 m_scale 进行缩放，用于模型失配实验。
    
    Attributes:
        m1, m2: 两个连杆的质量 (kg)
        l1, l2: 两个连杆的长度 (m)
        g: 重力加速度 (m/s²)
    
    Example:
        >>> arm = TwoLinkArm(m_scale=1.0)
        >>> M, C, G = arm.get_dynamics(q=np.array([0.5, 0.3]), dq=np.array([0.1, 0.2]))
    """
    
    def __init__(self, 
                 m1: float = 1.0,
                 m2: float = 1.0,
                 l1: float = 1.0,
                 l2: float = 1.0,
                 m_scale: float = 1.0):
        """
        初始化 2-DOF 机械臂模型。
        
        Args:
            m1: 第一连杆质量 (kg)，默认 1.0
            m2: 第二连杆质量 (kg)，默认 1.0
            l1: 第一连杆长度 (m)，默认 1.0
            l2: 第二连杆长度 (m)，默认 1.0
            m_scale: 质量缩放因子，用于模型失配实验。
                     例如 m_scale=0.7 表示控制器认为质量是真实值的 70%。
        """
        super().__init__(dof=2)
        
        # 应用质量缩放
        self.m1 = m1 * m_scale
        self.m2 = m2 * m_scale
        self.l1 = l1
        self.l2 = l2
        self.gravity = 9.81
        
        # 简化的转动惯量（假设质量集中在连杆末端）
        self.I1 = 0.5 * self.m1 * self.l1**2
        self.I2 = 0.5 * self.m2 * self.l2**2
    
    def get_dynamics(self, q: np.ndarray, dq: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        计算完整的 2-DOF 机械臂动力学矩阵。
        
        Args:
            q: 关节角度 [q1, q2]，shape (2,)
            dq: 关节角速度 [dq1, dq2]，shape (2,)
        
        Returns:
            M: 2x2 惯性矩阵（对称正定）
            C: 2x2 科氏力/离心力矩阵
            G: 长度为 2 的重力向量
        """
        m1, m2 = self.m1, self.m2
        l1, l2 = self.l1, self.l2
        g = self.gravity
        I1, I2 = self.I1, self.I2
        
        q1, q2 = q[0], q[1]
        dq1, dq2 = dq[0], dq[1]
        
        # 三角函数
        c2 = np.cos(q2)
        s2 = np.sin(q2)
        c1 = np.cos(q1)
        c12 = np.cos(q1 + q2)
        
        # ========================================
        # 惯性矩阵 M(q)
        # ========================================
        # M11 = m1*l1² + m2*(l1² + l2² + 2*l1*l2*cos(q2)) + I1 + I2
        # M12 = M21 = m2*(l2² + l1*l2*cos(q2)) + I2
        # M22 = m2*l2² + I2
        
        M11 = m1*l1**2 + m2*(l1**2 + l2**2 + 2*l1*l2*c2) + I1 + I2
        M12 = m2*(l2**2 + l1*l2*c2) + I2
        M22 = m2*l2**2 + I2
        
        M = np.array([
            [M11, M12],
            [M12, M22]
        ])
        
        # ========================================
        # 科氏力/离心力矩阵 C(q, dq)
        # ========================================
        # 使用 Christoffel 符号法构造，确保满足偏斜对称性
        h = -m2 * l1 * l2 * s2
        
        C = np.array([
            [h * dq2,       h * (dq1 + dq2)],
            [-h * dq1,      0.0]
        ])
        
        # ========================================
        # 重力向量 g(q)
        # ========================================
        g1 = (m1 + m2) * g * l1 * c1 + m2 * g * l2 * c12
        g2 = m2 * g * l2 * c12
        
        G = np.array([g1, g2])
        
        return M, C, G
