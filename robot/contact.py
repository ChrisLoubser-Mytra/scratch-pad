"""
Contact force model for guide wheels on rail flanges
"""

from typing import Tuple
import numpy as np

from robot.params import RobotParams
from robot.geometry import RailGeometry


class ContactModel:
    """Models contact forces between guide wheels and rail flanges"""
    
    def __init__(
        self,
        params: RobotParams,
        rail_geometry: RailGeometry,
        contact_stiffness: float = 1e6,
        contact_damping: float = 1000,
        friction_coefficient: float = 0.3
    ) -> None:
        """
        Initialize contact model
        
        Args:
            params: Robot physical parameters
            rail_geometry: Rail geometry including spacing
            contact_stiffness: Contact stiffness (N/m)
            contact_damping: Contact damping (N·s/m)
            friction_coefficient: Friction coefficient
        """
        self.params = params
        self.rail_geometry = rail_geometry
        self.contact_stiffness = contact_stiffness
        self.contact_damping = contact_damping
        self.friction_coefficient = friction_coefficient
        
        # Guide wheel positions (fixed, not centered)
        self.guide_wheel_left_pos = -params.guide_wheel_separation_across / 2
        self.guide_wheel_right_pos = params.guide_wheel_separation_across / 2
    
    def calculate_contact_force(
        self, x: float, y: float, vy: float, theta: float = 0.0
    ) -> Tuple[float, float, float, float]:
        """
        Calculate contact forces from guide wheels on rail flanges
        
        Args:
            x: Position along rail (m)
            y: Lateral position (m)
            vy: Lateral velocity (m/s)
            theta: Angular orientation (rad) - affects guide wheel positions
            
        Returns:
            Tuple of (force_left, force_right, penetration_left, penetration_right) in Newtons and meters
        """
        # Get flange positions at position x (accounting for rail angle/curvature)
        left_flange_pos, right_flange_pos = self.rail_geometry.get_flange_positions(x)
        
        guide_wheel_half_width = self.params.guide_wheel_width / 2
        
        # Account for robot's lateral position (y) and angular misalignment (theta)
        # Guide wheels are positioned between the two sets of drive wheels
        # With angular misalignment, guide wheels rotate around robot center
        # For small angles: cos(theta) ≈ 1, sin(theta) ≈ theta
        # Front/back position of guide wheels (between drive wheel sets)
        # Guide wheels are at guide_wheel_center_x (between wheel sets)
        # The angular misalignment causes the front and back guide wheels to have different lateral positions
        # Front guide wheel: y + guide_wheel_left_pos + (wheel_base/2) * theta
        # Back guide wheel: y + guide_wheel_left_pos - (wheel_base/2) * theta
        # For simplicity, use the center position (average of front and back)
        # This is approximately: y + guide_wheel_left_pos (for small angles)
        # But we need to account for the fact that rotation causes differential lateral positions
        
        # Guide wheel positions accounting for rotation
        # The guide wheels are at fixed lateral positions relative to robot center
        # When rotated, the front and back guide wheels have different lateral positions
        # Use the average (center) position for contact calculation
        guide_wheel_center_x = 0.0  # Guide wheels are centered between drive wheel sets
        left_wheel_center = y + self.guide_wheel_left_pos  # Simplified: use center position
        right_wheel_center = y + self.guide_wheel_right_pos
        
        # Guide wheel edges (outer edges that contact flanges)
        # Guide wheels are on the inside, so left wheel's right edge contacts left flange
        # Right wheel's left edge contacts right flange
        left_wheel_contact_edge = left_wheel_center + guide_wheel_half_width
        right_wheel_contact_edge = right_wheel_center - guide_wheel_half_width
        
        # Calculate gaps
        # The gap is the distance from the wheel contact edge to the flange
        # Positive gap = no contact (wheel is inside, gap exists)
        # Negative gap = contact (wheel has penetrated flange)
        # For left side: gap = left_flange_pos - left_wheel_contact_edge
        #   Positive gap = wheel edge is to the left of flange (no contact, wheel is inside)
        #   Negative gap = wheel edge is to the right of flange (contact, wheel has penetrated)
        # For right side: gap = right_wheel_contact_edge - right_flange_pos
        #   Positive gap = wheel edge is to the right of flange (no contact, wheel is inside)
        #   Negative gap = wheel edge is to the left of flange (contact, wheel has penetrated)
        gap_left = left_flange_pos - left_wheel_contact_edge
        gap_right = right_wheel_contact_edge - right_flange_pos
        
        # Contact forces (only when gap is negative, i.e., wheel penetrates flange)
        force_left = 0.0
        force_right = 0.0
        penetration_left = 0.0
        penetration_right = 0.0
        
        if gap_left < 0:  # Left wheel has penetrated left flange
            penetration_left = -gap_left
            # Limit penetration to flange height (climbing check)
            penetration_left = min(penetration_left, self.params.rail_flange_height)
            
            # Normal force (spring-damper) - adjust stiffness for large penetrations
            effective_stiffness = self.contact_stiffness
            if self.rail_geometry.spacing > 0.05:  # >50mm spacing
                # Reduce effective stiffness for larger gaps (less rigid contact)
                effective_stiffness = self.contact_stiffness * (0.05 / self.rail_geometry.spacing) ** 0.5
            
            normal_force = effective_stiffness * penetration_left + self.contact_damping * vy
            # Friction force (opposes motion)
            friction_force = (
                self.friction_coefficient * abs(normal_force) * np.sign(vy)
                if abs(vy) > 0.01
                else 0.0
            )
            force_left = normal_force + friction_force
        
        if gap_right < 0:  # Right wheel has penetrated right flange
            penetration_right = -gap_right
            # Limit penetration to flange height (climbing check)
            penetration_right = min(penetration_right, self.params.rail_flange_height)
            
            # Normal force (spring-damper)
            effective_stiffness = self.contact_stiffness
            if self.rail_geometry.spacing > 0.05:  # >50mm spacing
                effective_stiffness = self.contact_stiffness * (0.05 / self.rail_geometry.spacing) ** 0.5
            
            normal_force = effective_stiffness * penetration_right - self.contact_damping * vy
            # Friction force (opposes motion)
            friction_force = (
                self.friction_coefficient * abs(normal_force) * np.sign(-vy)
                if abs(vy) > 0.01
                else 0.0
            )
            force_right = normal_force + friction_force
        
        return force_left, force_right, penetration_left, penetration_right

