"""
Main robot simulator class
"""

from typing import Tuple
import numpy as np
from scipy.integrate import odeint

from robot.params import RobotParams
from robot.geometry import RailGeometry, WheelLayout
from robot.contact import ContactModel
from robot.dynamics import Dynamics
from robot.analysis import StabilityAnalyzer


class RobotSimulator:
    """Simulates robot dynamics on rails with guide wheels"""
    
    def __init__(
        self,
        params: RobotParams,
        spacing: float,
        initial_theta: float = 0.01,
        rail_angle: float = 0.0,
        rail_angle_per_meter: float = 0.0
    ) -> None:
        """
        Initialize simulator
        
        Args:
            params: Robot physical parameters
            spacing: Gap between guide wheels and rail flanges (m)
            initial_theta: Initial angular misalignment (rad)
            rail_angle: Constant rail angle (rad) - angle of rails relative to straight
            rail_angle_per_meter: Rail angle change per meter (rad/m) - curvature
        """
        self.params = params
        self.spacing = spacing
        self.initial_theta = initial_theta
        
        # Initialize components
        self.rail_geometry = RailGeometry(params, spacing, rail_angle, rail_angle_per_meter)
        self.wheel_layout = WheelLayout(params)
        self.contact_model = ContactModel(
            params,
            self.rail_geometry,
            contact_stiffness=1e6,
            contact_damping=1000,
            friction_coefficient=0.3
        )
        self.dynamics_obj = Dynamics(params, self.contact_model, spacing)
        self.analyzer = StabilityAnalyzer(params, spacing)
        
        # Store for compatibility (legacy interface for app.py)
        self.flange_separation = params.flange_separation_fixed
        self.guide_wheel_left_pos = -params.guide_wheel_separation_across / 2
        self.guide_wheel_right_pos = params.guide_wheel_separation_across / 2
        self.max_safe_contact_force = 50000.0
        self.climbing_force_threshold = 0.4
    
    @property
    def contact_stiffness(self) -> float:
        """Contact stiffness (legacy interface)"""
        return self.contact_model.contact_stiffness
    
    @property
    def contact_damping(self) -> float:
        """Contact damping (legacy interface)"""
        return self.contact_model.contact_damping
    
    @property
    def friction_coefficient(self) -> float:
        """Friction coefficient (legacy interface)"""
        return self.contact_model.friction_coefficient
    
    def contact_force(self, y: float, vy: float) -> Tuple[float, float, float, float]:
        """
        Calculate contact forces (legacy interface)
        
        Args:
            y: Lateral position (m)
            vy: Lateral velocity (m/s)
            
        Returns:
            Tuple of (force_left, force_right, penetration_left, penetration_right)
        """
        # Use x=0, theta=0 for legacy calls (assumes straight rails, no angular misalignment)
        return self.contact_model.calculate_contact_force(0.0, y, vy, 0.0)
    
    def dynamics(self, state: np.ndarray, t: float) -> np.ndarray:
        """
        System dynamics (legacy interface)
        
        Args:
            state: [x, y, theta, vx, vy, omega]
            t: Time
            
        Returns:
            Derivative of state vector
        """
        return self.dynamics_obj.calculate_dynamics(state, t)
    
    def simulate(
        self, duration: float = 10.0, dt: float = 0.001, max_distance: float = 10.0
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Run simulation
        
        Args:
            duration: Maximum simulation duration (s)
            dt: Time step (s)
            max_distance: Maximum distance to travel (m)
            
        Returns:
            Tuple of (time_array, state_history, contact_forces_history)
        """
        # Calculate time needed to travel max_distance
        t_accel = self.params.max_speed / self.params.acceleration
        d_accel = 0.5 * self.params.acceleration * t_accel**2
        
        if max_distance <= d_accel:
            actual_duration = np.sqrt(2 * max_distance / self.params.acceleration)
        else:
            d_constant = max_distance - d_accel
            t_constant = d_constant / self.params.max_speed
            actual_duration = t_accel + t_constant
        
        actual_duration = min(actual_duration, duration)
        t = np.arange(0, actual_duration, dt)
        
        # Initial state: start with offset to ensure contact occurs
        # Guide wheels are at ±559.6mm, flanges at ±609.6mm
        # To contact left flange: left_wheel_edge < left_flange
        # left_wheel_edge = y + guide_wheel_left + guide_wheel_half_width
        # y < flange_left - guide_wheel_left - guide_wheel_half_width
        # y < -609.6 - (-559.6) - 8 = -58mm (for left contact)
        # Similarly for right: y > 58mm (for right contact)
        
        # Initial state: start near center with small offset based on initial misalignment
        # The "spacing" parameter is the gap between guide wheel and flange when robot is centered
        # For small spacings (1mm), robot should start near center and drive straight
        # For larger spacings, start with small offset to ensure contact occurs
        # The initial offset should be based on the initial angular misalignment (theta)
        # With initial theta, the front and back of the robot are offset by: wheel_base * tan(theta)
        # This creates a lateral offset that will cause contact
        
        # Max offset before contact = spacing (the gap itself)
        max_offset = self.spacing
        
        # For very small spacings (1mm), start near center - robot should drive straight
        if self.spacing < 0.002:  # <2mm
            # Start very close to center, let the initial misalignment cause contact
            initial_y_offset = self.params.wheel_base * np.tan(self.initial_theta) * 0.5  # Half the geometric offset
        elif self.spacing < 0.01:  # 2-10mm
            # Small offset to ensure contact, but allow room for oscillation
            initial_y_offset = max_offset * 0.2  # 20% of spacing (allows oscillation)
        else:  # >=10mm
            # Larger offset for larger gaps, but still allow oscillation
            initial_y_offset = max_offset * 0.3  # 30% of spacing (allows oscillation)
        
        initial_state = np.array([
            0.0,  # x
            initial_y_offset,  # y
            self.initial_theta,  # theta
            0.0,  # vx
            0.0,  # vy
            0.0,  # omega
        ])
        
        # Integrate ODE
        solution = odeint(self.dynamics_obj.calculate_dynamics, initial_state, t)
        
        # Stop simulation early if max_distance reached
        x_positions = solution[:, 0]
        if np.any(x_positions >= max_distance):
            stop_idx = np.where(x_positions >= max_distance)[0][0] + 1
            solution = solution[:stop_idx]
            t = t[:stop_idx]
        
        # Calculate contact forces for each time step
        contact_forces = np.zeros((len(t), 4))
        for i in range(len(t)):
            x = solution[i, 0]
            y = solution[i, 1]
            theta = solution[i, 2]
            vy = solution[i, 4]
            force_left, force_right, pen_left, pen_right = self.contact_model.calculate_contact_force(
                x, y, vy, theta
            )
            contact_forces[i] = [force_left, force_right, pen_left, pen_right]
        
        return t, solution, contact_forces
    
    def analyze_stability(
        self, t: np.ndarray, state: np.ndarray, contact_forces: np.ndarray
    ) -> dict:
        """
        Analyze simulation results (legacy interface)
        
        Args:
            t: Time array
            state: State history
            contact_forces: Contact forces history
            
        Returns:
            Dictionary with analysis results
        """
        return self.analyzer.analyze(t, state, contact_forces)

