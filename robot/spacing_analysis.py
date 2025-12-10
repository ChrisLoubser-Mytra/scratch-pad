"""
Spacing analysis functions
"""

from typing import Dict, Any
from robot.params import RobotParams
from robot.simulator import RobotSimulator


def skew_mm_to_theta(skew_mm: float, wheel_base_m: float = 1.2) -> float:
    """
    Convert front-to-back skew in mm to angular misalignment in radians
    
    Args:
        skew_mm: Lateral offset between front and back of robot (mm)
        wheel_base_m: Distance between front and back wheels (m)
        
    Returns:
        Angular misalignment in radians
    """
    return skew_mm / 1000.0 / wheel_base_m


def run_spacing_analysis(
    spacings_mm: list[float],
    duration: float = 10.0,
    initial_skew_mm: float = 10.0,
    max_distance: float = 10.0,
    rail_angle: float = 0.0,
    rail_angle_per_meter: float = 0.0
) -> Dict[float, Dict[str, Any]]:
    """
    Run simulation for multiple spacing values
    
    Args:
        spacings_mm: List of spacing values in millimeters
        duration: Maximum simulation duration in seconds
        initial_skew_mm: Initial front-to-back skew in millimeters
        max_distance: Maximum distance to travel (m)
        rail_angle: Constant rail angle (rad) - angle of rails relative to straight
        rail_angle_per_meter: Rail angle change per meter (rad/m) - curvature
        
    Returns:
        Dictionary with results for each spacing
    """
    params = RobotParams()
    initial_theta = skew_mm_to_theta(initial_skew_mm, params.wheel_base)
    results: Dict[float, Dict[str, Any]] = {}
    
    for spacing_mm in spacings_mm:
        spacing = spacing_mm / 1000.0  # Convert to meters
        simulator = RobotSimulator(
            params,
            spacing,
            initial_theta=initial_theta,
            rail_angle=rail_angle,
            rail_angle_per_meter=rail_angle_per_meter
        )
        
        t, state, contact_forces = simulator.simulate(duration=duration, max_distance=max_distance)
        analysis = simulator.analyze_stability(t, state, contact_forces)
        
        results[spacing_mm] = {
            "time": t,
            "state": state,
            "contact_forces": contact_forces,
            "analysis": analysis,
            "simulator": simulator,
        }
    
    return results

