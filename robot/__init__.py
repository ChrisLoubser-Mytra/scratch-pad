"""
Mytra Robot Guide Wheel Spacing Simulation

This package simulates the dynamics of a warehouse robot traveling on rails,
analyzing the effect of guide wheel spacing on stability and ping-ponging behavior.
"""

from robot.params import RobotParams
from robot.state import SimulationState
from robot.simulator import RobotSimulator
from robot.spacing_analysis import run_spacing_analysis, skew_mm_to_theta

__all__ = [
    "RobotParams",
    "SimulationState",
    "RobotSimulator",
    "run_spacing_analysis",
    "skew_mm_to_theta",
]

