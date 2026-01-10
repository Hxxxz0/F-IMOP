"""
F-IMOP: Filtered-Error Inertia-Metric Optimal Projection for Safe Robot Control

A lightweight safety filter that guarantees exponential stability for robot
trajectory tracking by projecting nominal control inputs onto the set of
stabilizing controls using the inertia metric.
"""

from .controller import FIMOPController
from .dynamics import DynamicsModel, TwoLinkArm, SixLinkArm

__version__ = "0.1.0"
__all__ = ["FIMOPController", "DynamicsModel", "TwoLinkArm", "SixLinkArm"]
