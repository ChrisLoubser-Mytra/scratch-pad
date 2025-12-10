"""
Robot dynamics equations
"""

from typing import TYPE_CHECKING
import numpy as np

if TYPE_CHECKING:
    from robot.contact import ContactModel
    from robot.params import RobotParams


class Dynamics:
    """Robot dynamics calculations"""
    
    def __init__(
        self,
        params: "RobotParams",
        contact_model: "ContactModel",
        spacing: float
    ) -> None:
        """
        Initialize dynamics calculator
        
        Args:
            params: Robot physical parameters
            contact_model: Contact force model
            spacing: Gap between guide wheels and flanges (m)
        """
        self.params = params
        self.contact_model = contact_model
        self.spacing = spacing
    
    def calculate_dynamics(self, state: np.ndarray, t: float) -> np.ndarray:
        """
        System dynamics: d(state)/dt = f(state, t)
        
        Args:
            state: [x, y, theta, vx, vy, omega]
            t: Time
            
        Returns:
            Derivative of state vector
        """
        x, y, theta, vx, vy, omega = state
        total_mass = self.params.robot_mass + self.params.max_pallet_mass
        
        # Forward motion (simplified: constant acceleration up to max speed)
        if vx < self.params.max_speed:
            ax = min(self.params.acceleration, (self.params.max_speed - vx) / 0.1)
        else:
            ax = 0.0
        
        # Lateral dynamics from guide wheel contact
        force_left, force_right, _, _ = self.contact_model.calculate_contact_force(x, y, vy, theta)
        
        # Net lateral force
        f_y = force_right - force_left
        
        # Angular dynamics
        # Torque from lateral forces on guide wheels
        # The guide wheels are positioned between the two sets of drive wheels
        # There are 2 guide wheels on each side (front and back), separated by guide_wheel_separation
        # The lever arm for torque is half the guide wheel separation (front to back)
        # This creates a restoring torque that prevents excessive rotation
        guide_wheel_lever_arm = self.params.guide_wheel_separation / 2  # Distance from center to front/back guide wheel
        torque = (force_right - force_left) * guide_wheel_lever_arm
        
        # Add geometric constraint: maximum angular misalignment is limited by flange contact
        # The "spacing" parameter is the gap between guide wheel and flange when robot is centered
        # To contact a flange, the robot must move laterally by the spacing amount
        # Max angle = arctan(spacing / wheel_base)
        # This is the maximum angle before the front/back guide wheels contact flanges
        max_offset = self.spacing  # The gap itself is the max offset before contact
        max_angle = np.arctan(max_offset / self.params.wheel_base) if max_offset > 0 else 0.0
        
        # Add restoring torque when angle exceeds geometric limit
        # This prevents the robot from rotating more than physically possible
        if abs(theta) > max_angle:
            # Strong restoring torque to bring angle back within limits
            angle_excess = abs(theta) - max_angle
            restoring_torque = -1e6 * angle_excess * np.sign(theta)  # Very strong restoring torque
            torque += restoring_torque
        elif abs(theta) > max_angle * 0.8:  # Soft constraint when approaching limit
            # Soft restoring torque as we approach the limit
            angle_ratio = abs(theta) / max_angle
            restoring_torque = -1e4 * (angle_ratio - 0.8) * np.sign(theta)
            torque += restoring_torque
        
        # Angular damping - prevents unbounded spinning
        base_damping = 200.0  # N·m·s/rad (base linear damping)
        if self.spacing > 0.05:  # Increase damping for larger spacings
            base_damping = 300.0 + 1000.0 * self.spacing
        angular_damping_linear = base_damping
        angular_damping_quadratic = 1000.0  # N·m·s²/rad²
        damping_torque = -angular_damping_linear * omega - angular_damping_quadratic * omega * abs(omega)
        
        # Angular acceleration
        alpha = (torque + damping_torque) / self.params.moment_of_inertia
        
        # Coupling: lateral motion affects orientation and vice versa
        # If moving forward with misalignment, there's a coupling
        coupling_force = -total_mass * vx * omega  # Centripetal-like effect
        
        # Limit lateral acceleration to ensure lateral velocity stays reasonable
        # Lateral velocity should not exceed forward velocity significantly
        # Apply damping to lateral motion to prevent excessive lateral velocities
        # Industry standard: lateral velocity should be < 30% of forward velocity for stability
        # Increased damping for better stability (based on rail-guided vehicle standards)
        lateral_damping = 200.0  # N·s/m (damping for lateral motion - increased for stability)
        if vx > 0.1:  # Only apply when moving forward
            max_lateral_velocity_ratio = 0.3  # Industry standard: max 30% of forward velocity
            if abs(vy) > abs(vx) * max_lateral_velocity_ratio:
                # Apply strong damping when lateral velocity exceeds threshold
                excess_velocity = abs(vy) - abs(vx) * max_lateral_velocity_ratio
                lateral_damping_force = -lateral_damping * vy - 500.0 * excess_velocity * np.sign(vy)  # Strong damping
            else:
                lateral_damping_force = -lateral_damping * vy  # Standard damping
        else:
            lateral_damping_force = -lateral_damping * vy  # Standard damping when not moving forward
        
        return np.array([
            vx,  # dx/dt
            vy,  # dy/dt
            omega,  # dtheta/dt
            ax,  # dvx/dt
            (f_y + coupling_force + lateral_damping_force) / total_mass,  # dvy/dt
            alpha,  # domega/dt
        ])

