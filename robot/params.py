"""
Robot physical parameters
"""

from dataclasses import dataclass


@dataclass
class RobotParams:
    """Physical parameters of the robot"""

    robot_mass: float = 227.0  # kg (500 lbs)
    max_pallet_mass: float = 1361.0  # kg (3000 lbs)
    # Drive wheels (8 total: 4 wheels in a single line on each side)
    drive_wheel_diameter: float = 0.1  # m (100mm)
    drive_wheel_width: float = 0.0381  # m (38.1mm)
    wheel_spacing_in_set: float = 0.105  # m (105mm, distance between wheels in a set)
    wheel_set_separation: float = 0.749  # m (749mm, distance between first wheels of sets on same side)
    # Guide wheels (horizontal, contact flanges)
    guide_wheel_diameter: float = 0.08  # m (80mm)
    guide_wheel_width: float = 0.016  # m (16mm, width of guide wheel contacting flange)
    guide_wheel_separation: float = 0.464  # m (464mm, distance between guide wheels on same side)
    # Robot body dimensions
    robot_body_height: float = 0.192  # m (192mm tall)
    robot_body_clearance: float = 0.02  # m (20mm above bottom of wheels)
    # Legacy parameters (kept for compatibility, calculated from above)
    wheel_diameter: float = 0.1  # m (100mm, same as drive_wheel_diameter)
    max_speed: float = 1.5  # m/s
    acceleration: float = 0.75  # m/s²
    wheel_base: float = 1.2  # m (1200mm, distance between outside faces of wheels)
    moment_of_inertia: float = 0.0  # Will be calculated
    # Rail dimensions
    rail_flange_height: float = 0.01905  # m (0.75 inches = 19.05mm vertical flange height)
    rail_horizontal_width: float = 0.06604  # m (2.6 inches = 66.04mm horizontal surface width)
    guide_wheel_separation_across: float = 1.1192  # m (1119.2mm, distance between guide wheels on left and right sides)
    flange_separation_fixed: float = 1.2192  # m (48 inches = 1219.2mm, fixed distance between vertical flanges)

    def __post_init__(self) -> None:
        """Calculate derived parameters"""
        total_mass = self.robot_mass + self.max_pallet_mass
        # Approximate moment of inertia for rectangular body
        # I = m * (w² + h²) / 12, assuming roughly square pallet
        # Using wheel_base as the length for inertia calculation
        self.moment_of_inertia = total_mass * (self.wheel_base**2) / 12

