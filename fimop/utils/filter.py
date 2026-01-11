"""
Low-Pass Filter Utilities for F-IMOP.

Provides a simple exponential moving average (EMA) filter to smooth
noisy observations before they are used by the F-IMOP controller.
"""

import numpy as np
from typing import Optional


class LowPassFilter:
    """
    一阶低通滤波器（指数移动平均）。
    
    公式: y[k] = alpha * x[k] + (1 - alpha) * y[k-1]
    
    其中:
        - alpha = 1.0 表示无滤波（输出 = 输入）
        - alpha -> 0 表示强滤波（输出变化缓慢）
    
    Attributes:
        alpha: 滤波系数 (0, 1]
        dim: 信号维度
        state: 当前滤波状态
    
    Example:
        >>> filt = LowPassFilter(alpha=0.1, dim=2)
        >>> for _ in range(10):
        ...     noisy_signal = np.array([1.0, 2.0]) + np.random.randn(2) * 0.1
        ...     filtered = filt.update(noisy_signal)
        >>> print(filtered)  # Smoothed result close to [1.0, 2.0]
    """
    
    def __init__(self, alpha: float = 0.1, dim: int = 1):
        """
        初始化滤波器。
        
        Args:
            alpha: 滤波系数。1.0 = 无滤波，0.05 = 强滤波。
                   经验公式: alpha ≈ 2π * fc * dt，其中 fc 是截止频率，dt 是采样周期。
            dim: 输入信号的维度。
        """
        if not 0 < alpha <= 1.0:
            raise ValueError(f"alpha must be in (0, 1], got {alpha}")
        
        self.alpha = alpha
        self.dim = dim
        self.state: Optional[np.ndarray] = None
    
    def update(self, value: np.ndarray) -> np.ndarray:
        """
        更新滤波器状态并返回滤波后的值。
        
        Args:
            value: 输入信号 [dim]
        
        Returns:
            滤波后的信号 [dim]
        """
        value = np.asarray(value, dtype=np.float64)
        
        if self.state is None:
            # 首次调用，直接使用输入值初始化
            self.state = value.copy()
        else:
            # EMA 更新
            self.state = self.alpha * value + (1 - self.alpha) * self.state
        
        return self.state.copy()
    
    def reset(self, initial_state: Optional[np.ndarray] = None):
        """
        重置滤波器状态。
        
        Args:
            initial_state: 可选的初始状态。若为 None，则下次 update 时自动初始化。
        """
        if initial_state is not None:
            self.state = np.asarray(initial_state, dtype=np.float64).copy()
        else:
            self.state = None
    
    def get_state(self) -> Optional[np.ndarray]:
        """返回当前滤波器状态的副本。"""
        return self.state.copy() if self.state is not None else None
    
    def __repr__(self) -> str:
        return f"LowPassFilter(alpha={self.alpha}, dim={self.dim})"
