"""
URDFModel: Dynamics model loader for URDF files using Pinocchio.

Requires 'pinocchio' library: pip install pin
"""

import numpy as np
import os
from typing import Tuple, Optional
from .base import DynamicsModel

try:
    import pinocchio as pin
except ImportError:
    pin = None


class URDFModel(DynamicsModel):
    """
    基于 URDF 的通用机器人动力学模型 (使用 Pinocchio 引擎)。
    
    使用高性能刚体动力学库 Pinocchio 加载 URDF 文件，并计算
    M, C, G 矩阵。
    
    Usage:
        >>> from fimop.dynamics import URDFModel
        >>> robot = URDFModel("path/to/robot.urdf")
        >>> M, C, g = robot.get_dynamics(q, dq)
    
    Attributes:
        model: Pinocchio 模型对象
        data: Pinocchio 数据对象
        
    Note:
        需要安装 pinocchio: `pip install pin`
    """
    
    def __init__(self, urdf_path: str, root_joint=None):
        """
        初始化 URDF 模型。
        
        Args:
            urdf_path: URDF 文件路径
            root_joint: 根关节类型 (e.g., pin.JointModelFreeFlyer() 用于浮动基座)
                        默认为 None (固定基座)
        
        Raises:
            ImportError: 如果未安装 pinocchio
            FileNotFoundError: 如果 URDF 文件不存在
        """
        if pin is None:
            raise ImportError(
                "Pinocchio library not found. "
                "Please install it via `pip install pin` to use URDFModel."
            )
            
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF file not found: {urdf_path}")
            
        # 加载模型
        if root_joint is None:
            self.model = pin.buildModelFromUrdf(urdf_path)
        else:
            self.model = pin.buildModelFromUrdf(urdf_path, root_joint)
            
        self.data = self.model.createData()
        
        # 初始化基类
        super().__init__(dof=self.model.nq)
        
    def get_dynamics(self, q: np.ndarray, dq: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        计算动力学矩阵 M, C, G。
        
        Args:
            q: 关节位置
            dq: 关节速度
            
        Returns:
            M: 惯性矩阵 (n, n)
            C: 科氏力/离心力矩阵 (n, n)
            G: 重力向量 (n,)
        """
        # 确保输入维度正确
        if q.shape[0] != self.model.nq or dq.shape[0] != self.model.nv:
            raise ValueError(
                f"State dimension mismatch. Expected q[{self.model.nq}], dq[{self.model.nv}], "
                f"got q[{q.shape[0]}], dq[{dq.shape[0]}]"
            )
            
        # 1. 计算 M (CRBA)
        # Pinocchio 的 crba 会更新 data.M
        pin.crba(self.model, self.data, q)
        
        # data.M 在 Python 中通常是 numpy 数组
        # Pinocchio 只计算上三角部分，我们需要手动对称化
        M = np.array(self.data.M)
        M = np.triu(M) + np.triu(M, 1).T

        # 2. 计算 C (Coriolis Matrix)
        # computeCoriolisMatrix returns C s.t. C*dq = Coriolis term
        pin.computeCoriolisMatrix(self.model, self.data, q, dq)
        C = self.data.C.copy()
        
        # 3. 计算 G (Gravity)
        pin.computeGeneralizedGravity(self.model, self.data, q)
        G = self.data.g.copy()
        
        return M, C, G
    
    def get_forward_kinematics(self, q: np.ndarray, frame_name: str) -> Tuple[np.ndarray, np.ndarray]:
        """
        获取指定坐标系的末端位姿 (位置, 旋转)。
        
        Args:
            q: 关节位置
            frame_name: 坐标系名称 (Link name in URDF)
            
        Returns:
            pos: (3,) 位置
            rot: (3, 3) 旋转矩阵
        """
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        
        frame_id = self.model.getFrameId(frame_name)
        if frame_id >= len(self.model.frames):
            raise ValueError(f"Frame '{frame_name}' not found.")
            
        frame = self.data.oMf[frame_id]
        return frame.translation.copy(), frame.rotation.copy()
