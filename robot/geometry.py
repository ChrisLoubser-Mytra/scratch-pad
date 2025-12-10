"""
Robot wheel layout and rail geometry calculations
"""

from typing import Tuple
import numpy as np

from robot.params import RobotParams


class WheelLayout:
    """Calculates wheel positions and maximum offset based on wheel layout"""

    def __init__(self, params: RobotParams) -> None:
        """
        Initialize wheel layout calculator

        Args:
            params: Robot physical parameters
        """
        self.params = params
        
        # Calculate drive wheel positions (4 wheels in a line on each side)
        # Front wheel at -wheel_base/2, rear wheel at +wheel_base/2
        # Two sets: first set (2 wheels), gap, second set (2 wheels)
        self.wheel1_x = -params.wheel_base / 2  # Front wheel center
        self.wheel2_x = self.wheel1_x + params.wheel_spacing_in_set  # Second wheel center
        self.wheel3_x = self.wheel2_x + params.wheel_set_separation  # Third wheel center (start of second set)
        self.wheel4_x = self.wheel3_x + params.wheel_spacing_in_set  # Fourth wheel center
        
        # Guide wheels positioned between the two sets (between wheel2 and wheel3)
        self.guide_wheel_center_x = (self.wheel2_x + self.wheel3_x) / 2
        
        # Guide wheel positions (fixed, not centered)
        self.guide_wheel_left_y = -params.guide_wheel_separation_across / 2
        self.guide_wheel_right_y = params.guide_wheel_separation_across / 2
        
        # Guide wheels on each side are 464mm apart (front to back)
        self.guide_wheel_left_front_x = self.guide_wheel_center_x - params.guide_wheel_separation / 2
        self.guide_wheel_left_rear_x = self.guide_wheel_center_x + params.guide_wheel_separation / 2
        self.guide_wheel_right_front_x = self.guide_wheel_center_x - params.guide_wheel_separation / 2
        self.guide_wheel_right_rear_x = self.guide_wheel_center_x + params.guide_wheel_separation / 2
        
        # Calculate maximum lateral offset based on wheel spacing
        # The robot can offset until drive wheels start to lose contact with rails
        # Maximum offset is limited by the wheel base and rail width
        # For a robot with wheel base L, max offset ≈ (rail_width - wheel_width) / 2
        # But also consider that guide wheels must stay within flanges
        rail_width = params.rail_horizontal_width
        wheel_width = params.drive_wheel_width
        self.max_drive_wheel_offset = (rail_width - wheel_width) / 2 if rail_width > wheel_width else 0.0
        
        # Maximum offset also limited by guide wheel contact
        # Guide wheels are 1119.2mm apart, flanges are 1219.2mm apart
        # So max offset before guide wheels lose contact = (1219.2 - 1119.2) / 2 = 50mm
        # But this is the spacing parameter, so max offset depends on spacing
        # For now, use a conservative estimate based on wheel base geometry
        # Max offset ≈ wheel_base * sin(max_angle), where max_angle is limited by wheel contact
        # Simplified: max offset is limited by the smaller of:
        # 1. Drive wheel contact with rail (rail width constraint)
        # 2. Guide wheel contact with flange (spacing constraint)
        self.max_offset_base = min(self.max_drive_wheel_offset, 0.05)  # Conservative 50mm base
    
    def get_max_offset(self, spacing: float) -> float:
        """
        Calculate maximum lateral offset for given spacing
        
        Args:
            spacing: Gap between guide wheels and flanges when robot is centered (m)
            
        Returns:
            Maximum lateral offset (m) before guide wheels contact flanges
        """
        # The spacing parameter IS the gap when robot is centered
        # To contact a flange, robot must move laterally by the spacing amount
        # So max offset = spacing (the gap itself)
        return spacing
    
    def get_drive_wheel_positions(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get drive wheel x-positions for left and right sides
        
        Returns:
            Tuple of (left_wheel_x_positions, right_wheel_x_positions)
            Both are arrays of 4 x-positions
        """
        wheel_x = np.array([self.wheel1_x, self.wheel2_x, self.wheel3_x, self.wheel4_x])
        return wheel_x, wheel_x  # Same x-positions for both sides
    
    def get_guide_wheel_positions(self) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        """
        Get guide wheel positions (x, y) for left and right sides
        
        Returns:
            Tuple of ((left_front_x, left_front_y), (left_rear_x, left_rear_y)),
                    ((right_front_x, right_front_y), (right_rear_x, right_rear_y))
        """
        left_front = (self.guide_wheel_left_front_x, self.guide_wheel_left_y)
        left_rear = (self.guide_wheel_left_rear_x, self.guide_wheel_left_y)
        right_front = (self.guide_wheel_right_front_x, self.guide_wheel_right_y)
        right_rear = (self.guide_wheel_right_rear_x, self.guide_wheel_right_y)
        return (left_front, left_rear), (right_front, right_rear)


class RailGeometry:
    """Rail geometry including straightness/angle"""
    
    def __init__(
        self,
        params: RobotParams,
        spacing: float,
        rail_angle: float = 0.0,
        rail_angle_per_meter: float = 0.0
    ) -> None:
        """
        Initialize rail geometry
        
        Args:
            params: Robot physical parameters
            spacing: Gap between guide wheels and flanges (m)
            rail_angle: Constant rail angle (rad) - angle of rails relative to straight
            rail_angle_per_meter: Rail angle change per meter (rad/m) - curvature
        """
        self.params = params
        self.spacing = spacing
        self.rail_angle = rail_angle  # Constant angle
        self.rail_angle_per_meter = rail_angle_per_meter  # Curvature
        
        # Fixed geometry
        self.flange_separation = params.flange_separation_fixed  # 1219.2mm fixed
        self.guide_wheel_left_pos = -params.guide_wheel_separation_across / 2
        self.guide_wheel_right_pos = params.guide_wheel_separation_across / 2
    
    def get_flange_positions(self, x: float) -> Tuple[float, float]:
        """
        Get left and right flange positions at position x along rail
        
        Args:
            x: Position along rail (m)
            
        Returns:
            Tuple of (left_flange_y, right_flange_y) in meters
        """
        # The "spacing" parameter is the gap between guide wheel EDGE and flange when robot is centered
        # Guide wheel positions (fixed relative to robot center)
        guide_wheel_left_center = -self.params.guide_wheel_separation_across / 2  # -559.6mm
        guide_wheel_right_center = self.params.guide_wheel_separation_across / 2  # +559.6mm
        guide_wheel_half_width = self.params.guide_wheel_width / 2  # 8mm
        
        # Guide wheel contact edges (outer edges that contact flanges)
        left_contact_edge = guide_wheel_left_center + guide_wheel_half_width  # -551.6mm
        right_contact_edge = guide_wheel_right_center - guide_wheel_half_width  # +551.6mm
        
        # When robot is centered (y=0), gap = spacing
        # Left: left_flange - left_contact_edge = spacing
        # Right: right_contact_edge - right_flange = spacing
        # So effective flange positions are:
        left_flange_base = left_contact_edge + self.spacing
        right_flange_base = right_contact_edge - self.spacing
        
        # Apply rail angle/curvature
        # If rails are angled, flanges shift laterally
        # Positive angle means rails curve to the right
        angle_at_x = self.rail_angle + self.rail_angle_per_meter * x
        lateral_shift = np.tan(angle_at_x) * x  # Approximate for small angles
        
        # For angled rails, the flange positions shift
        left_flange_y = left_flange_base + lateral_shift
        right_flange_y = right_flange_base + lateral_shift
        
        return left_flange_y, right_flange_y
    
    def get_rail_angle_at(self, x: float) -> float:
        """
        Get rail angle at position x
        
        Args:
            x: Position along rail (m)
            
        Returns:
            Rail angle in radians
        """
        return self.rail_angle + self.rail_angle_per_meter * x

