"""
Simulation state representation
"""

from dataclasses import dataclass


@dataclass
class SimulationState:
    """State vector for the simulation"""

    x: float  # Position along rail (m)
    y: float  # Lateral position (m)
    theta: float  # Angular orientation (rad)
    vx: float  # Velocity along rail (m/s)
    vy: float  # Lateral velocity (m/s)
    omega: float  # Angular velocity (rad/s)

