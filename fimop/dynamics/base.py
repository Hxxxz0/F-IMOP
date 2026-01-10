"""
DynamicsModel: Abstract base class for robot dynamics.

Users should inherit from this class and implement get_dynamics() to use
F-IMOP with their own robots.
"""

from abc import ABC, abstractmethod
from typing import Tuple
import numpy as np


class DynamicsModel(ABC):
    """
    机器人动力学模型抽象基类。
    
    任何机器人只要实现了这个接口，就可以使用 F-IMOP 安全控制器。
    该接口要求提供标准的刚体动力学表示：M(q)ẍ + C(q,q̇)q̇ + g(q) = τ
    
    重要约束：
    1. M(q) 必须是对称正定矩阵
    2. C(q, q̇) 应满足偏斜对称性: x^T (Ṁ - 2C) x = 0
       这是大多数刚体机器人的固有属性，确保了能量守恒
    
    Example:
        >>> class MyRobot(DynamicsModel):
        ...     def __init__(self):
        ...         super().__init__(dof=7)
        ...     
        ...     def get_dynamics(self, q, dq):
        ...         # 使用 Pinocchio, PyBullet 等计算
        ...         M = compute_inertia(q)
        ...         C = compute_coriolis(q, dq)
        ...         g = compute_gravity(q)
        ...         return M, C, g
    """
    
    def __init__(self, dof: int):
        """
        初始化动力学模型。
        
        Args:
            dof: 机器人自由度（关节数量）
        """
        if dof <= 0:
            raise ValueError(f"DOF must be positive, got {dof}")
        self._dof = dof
    
    @property
    def dof(self) -> int:
        """返回机器人的自由度。"""
        return self._dof
    
    @abstractmethod
    def get_dynamics(self, q: np.ndarray, dq: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        核心动力学接口：计算当前状态下的动力学矩阵。
        
        这是必须实现的方法。返回的矩阵将用于 F-IMOP 的投影计算。
        
        Args:
            q: 关节位置向量，shape (n,)
            dq: 关节速度向量，shape (n,)
        
        Returns:
            M: 惯性矩阵 M(q)，shape (n, n)
               必须是对称正定矩阵
            C: 科氏力与离心力矩阵 C(q, q̇)，shape (n, n)
               需满足 x^T(Ṁ - 2C)x = 0 的偏斜对称性
            g: 重力向量 g(q)，shape (n,)
        
        Raises:
            NotImplementedError: 子类必须实现此方法
        """
        raise NotImplementedError("Subclasses must implement get_dynamics()")
    
    def get_mass_matrix(self, q: np.ndarray) -> np.ndarray:
        """
        仅获取惯性矩阵 M(q)。
        
        默认实现调用 get_dynamics()，但子类可以重写以优化性能
        （例如，如果计算 M 比完整动力学更快）。
        
        Args:
            q: 关节位置向量
        
        Returns:
            M: 惯性矩阵，shape (n, n)
        """
        M, _, _ = self.get_dynamics(q, np.zeros(self._dof))
        return M
    
    def get_gravity(self, q: np.ndarray) -> np.ndarray:
        """
        仅获取重力向量 g(q)。
        
        Args:
            q: 关节位置向量
        
        Returns:
            g: 重力向量，shape (n,)
        """
        _, _, g = self.get_dynamics(q, np.zeros(self._dof))
        return g
    
    def forward_dynamics(self, q: np.ndarray, dq: np.ndarray, tau: np.ndarray) -> np.ndarray:
        """
        正向动力学：给定力矩计算加速度。
        
        Args:
            q: 关节位置
            dq: 关节速度
            tau: 关节力矩
        
        Returns:
            ddq: 关节加速度，shape (n,)
        """
        M, C, g = self.get_dynamics(q, dq)
        ddq = np.linalg.solve(M, tau - C @ dq - g)
        return ddq
    
    def inverse_dynamics(self, q: np.ndarray, dq: np.ndarray, ddq: np.ndarray) -> np.ndarray:
        """
        逆向动力学：给定加速度计算力矩。
        
        Args:
            q: 关节位置
            dq: 关节速度
            ddq: 期望关节加速度
        
        Returns:
            tau: 所需关节力矩，shape (n,)
        """
        M, C, g = self.get_dynamics(q, dq)
        tau = M @ ddq + C @ dq + g
        return tau
    
    def __repr__(self) -> str:
        return f"{self.__class__.__name__}(dof={self._dof})"
