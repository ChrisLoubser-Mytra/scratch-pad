"""
Mytra Robot Guide Wheel Spacing Simulation

This module simulates the dynamics of a warehouse robot traveling on rails,
analyzing the effect of guide wheel spacing on stability and ping-ponging behavior.
"""

from dataclasses import dataclass
from typing import Tuple, Dict, Any

import numpy as np
from scipy.integrate import odeint


@dataclass
class RobotParams:
    """Physical parameters of the robot"""

    robot_mass: float = 227.0  # kg (500 lbs)
    max_pallet_mass: float = 1361.0  # kg (3000 lbs)
    wheel_diameter: float = 0.3  # m (300mm)
    guide_wheel_width: float = 0.07  # m (70mm)
    max_speed: float = 1.5  # m/s
    acceleration: float = 0.75  # m/s²
    wheel_base: float = 0.96  # m (48 inches, approximate wheel spacing)
    moment_of_inertia: float = 0.0  # Will be calculated

    def __post_init__(self) -> None:
        """Calculate derived parameters"""
        total_mass = self.robot_mass + self.max_pallet_mass
        # Approximate moment of inertia for rectangular body
        # I = m * (w² + h²) / 12, assuming roughly square pallet
        self.moment_of_inertia = total_mass * (self.wheel_base**2) / 12


@dataclass
class SimulationState:
    """State vector for the simulation"""

    x: float  # Position along rail (m)
    y: float  # Lateral position (m)
    theta: float  # Angular orientation (rad)
    vx: float  # Velocity along rail (m/s)
    vy: float  # Lateral velocity (m/s)
    omega: float  # Angular velocity (rad/s)


class RobotSimulator:
    """Simulates robot dynamics on rails with guide wheels"""

    def __init__(
        self, params: RobotParams, spacing: float, initial_theta: float = 0.01
    ) -> None:
        """
        Initialize simulator

        Args:
            params: Robot physical parameters
            spacing: Gap between guide wheels and rail flanges (m)
            initial_theta: Initial angular misalignment (rad)
        """
        self.params = params
        self.spacing = spacing  # Gap in meters
        self.initial_theta = initial_theta
        self.rail_width = 0.07 + 2 * spacing  # Total rail width including gaps

        # Contact force parameters
        self.contact_stiffness = 1e6  # N/m (stiff contact)
        self.contact_damping = 1000  # N·s/m (damping)
        self.friction_coefficient = 0.3  # Friction between guide wheel and rail
        
    def contact_force(self, y: float, vy: float) -> Tuple[float, float]:
        """
        Calculate contact forces from guide wheels on rail flanges

        Args:
            y: Lateral position (m)
            vy: Lateral velocity (m/s)

        Returns:
            Tuple of (force_left, force_right) in Newtons
        """
        # Rail center is at y=0
        # Left flange at y = -rail_width/2, right flange at y = +rail_width/2
        # Guide wheels are positioned at the edges of the guide wheel width
        # The guide wheel outer edges are at y ± guide_wheel_width/2

        guide_wheel_half_width = self.params.guide_wheel_width / 2
        left_wheel_edge = y - guide_wheel_half_width
        right_wheel_edge = y + guide_wheel_half_width

        left_flange_pos = -self.rail_width / 2
        right_flange_pos = self.rail_width / 2

        # Calculate gaps: positive gap means wheel is inside (no contact)
        # Negative gap means wheel has penetrated flange (contact)
        gap_left = left_wheel_edge - left_flange_pos  # Positive if wheel edge is to the right of flange
        gap_right = right_flange_pos - right_wheel_edge  # Positive if wheel edge is to the left of flange

        # Contact forces (only when gap is negative, i.e., wheel penetrates flange)
        force_left = 0.0
        force_right = 0.0

        if gap_left < 0:  # Left wheel edge has penetrated left flange
            penetration = -gap_left
            # Normal force (spring-damper)
            normal_force = self.contact_stiffness * penetration + self.contact_damping * vy
            # Friction force (opposes motion)
            friction_force = (
                self.friction_coefficient * abs(normal_force) * np.sign(vy)
                if abs(vy) > 0.01
                else 0.0
            )
            force_left = normal_force + friction_force

        if gap_right < 0:  # Right wheel edge has penetrated right flange
            penetration = -gap_right
            # Normal force (spring-damper)
            normal_force = self.contact_stiffness * penetration - self.contact_damping * vy
            # Friction force (opposes motion)
            friction_force = (
                self.friction_coefficient * abs(normal_force) * np.sign(-vy)
                if abs(vy) > 0.01
                else 0.0
            )
            force_right = normal_force + friction_force

        return force_left, force_right
    
    def dynamics(self, state: np.ndarray, t: float) -> np.ndarray:
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
        force_left, force_right = self.contact_force(y, vy)

        # Net lateral force
        f_y = force_right - force_left

        # Angular dynamics
        # Torque from lateral forces (lever arm is half the guide wheel width)
        torque = (force_right - force_left) * self.params.guide_wheel_width / 2

        # Angular acceleration
        alpha = torque / self.params.moment_of_inertia

        # Coupling: lateral motion affects orientation and vice versa
        # If moving forward with misalignment, there's a coupling
        coupling_force = -total_mass * vx * omega  # Centripetal-like effect

        return np.array([
            vx,  # dx/dt
            vy,  # dy/dt
            omega,  # dtheta/dt
            ax,  # dvx/dt
            (f_y + coupling_force) / total_mass,  # dvy/dt
            alpha,  # domega/dt
        ])
    
    def simulate(
        self, duration: float = 10.0, dt: float = 0.001
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Run simulation

        Args:
            duration: Simulation duration (s)
            dt: Time step (s)

        Returns:
            Tuple of (time_array, state_history)
        """
        t = np.arange(0, duration, dt)

        # Initial state: slight misalignment
        initial_state = np.array([
            0.0,  # x
            0.0,  # y (centered)
            self.initial_theta,  # theta (small initial misalignment)
            0.0,  # vx
            0.0,  # vy
            0.0,  # omega
        ])

        # Integrate ODE
        solution = odeint(self.dynamics, initial_state, t)

        return t, solution
    
    def analyze_stability(self, t: np.ndarray, state: np.ndarray) -> Dict[str, Any]:
        """
        Analyze simulation results for ping-ponging behavior

        Args:
            t: Time array
            state: State history [N x 6]

        Returns:
            Dictionary with analysis results
        """
        y = state[:, 1]  # Lateral position
        theta = state[:, 2]  # Angular orientation
        vy = state[:, 4]  # Lateral velocity
        omega = state[:, 5]  # Angular velocity

        # Calculate statistics
        y_std = float(np.std(y))
        y_max = float(np.max(np.abs(y)))
        theta_std = float(np.std(theta))
        theta_max = float(np.max(np.abs(theta)))

        # Detect oscillations (ping-ponging)
        # Count zero crossings in lateral velocity
        vy_sign_changes = int(np.sum(np.diff(np.sign(vy)) != 0))
        oscillation_frequency = float(vy_sign_changes / (2 * t[-1]) if t[-1] > 0 else 0.0)

        # Check if oscillations are growing (unstable) or decaying (stable)
        # Analyze amplitude over time windows
        n_windows = 5
        window_size = len(y) // n_windows
        amplitudes: list[float] = []

        for i in range(n_windows):
            start_idx = i * window_size
            end_idx = (i + 1) * window_size if i < n_windows - 1 else len(y)
            window_y = y[start_idx:end_idx]
            amplitudes.append(float(np.max(np.abs(window_y))))

        # Determine if oscillations are growing
        if len(amplitudes) >= 2:
            amplitude_trend = float(np.polyfit(range(len(amplitudes)), amplitudes, 1)[0])
            is_growing = amplitude_trend > 0.001  # Growing if trend > 1mm
        else:
            amplitude_trend = 0.0
            is_growing = False

        # Determine stability
        # Ping-ponging occurs if: oscillations exist AND they're not decaying quickly
        is_ping_ponging = (oscillation_frequency > 0.5) and (y_max > self.spacing * 2)

        return {
            "lateral_std": y_std,
            "lateral_max": y_max,
            "angular_std": theta_std,
            "angular_max": theta_max,
            "oscillation_frequency": oscillation_frequency,
            "is_ping_ponging": is_ping_ponging,
            "is_growing": is_growing,
            "amplitude_trend": amplitude_trend,
            "zero_crossings": vy_sign_changes,
        }


def run_spacing_analysis(
    spacings_mm: list[float], duration: float = 10.0, initial_theta: float = 0.01
) -> Dict[float, Dict[str, Any]]:
    """
    Run simulation for multiple spacing values

    Args:
        spacings_mm: List of spacing values in millimeters
        duration: Simulation duration in seconds
        initial_theta: Initial angular misalignment in radians

    Returns:
        Dictionary with results for each spacing
    """
    params = RobotParams()
    results: Dict[float, Dict[str, Any]] = {}

    for spacing_mm in spacings_mm:
        spacing = spacing_mm / 1000.0  # Convert to meters
        simulator = RobotSimulator(params, spacing, initial_theta=initial_theta)

        t, state = simulator.simulate(duration=duration)
        analysis = simulator.analyze_stability(t, state)

        results[spacing_mm] = {
            "time": t,
            "state": state,
            "analysis": analysis,
            "simulator": simulator,
        }

    return results


if __name__ == "__main__":
    # Test simulation
    spacings = [5, 10, 20, 30, 40]  # mm
    results = run_spacing_analysis(spacings, duration=10.0)

    # Print results
    print("Spacing Analysis Results:")
    print("-" * 60)
    for spacing_mm, data in results.items():
        analysis = data["analysis"]
        print(f"\nSpacing: {spacing_mm}mm")
        print(f"  Ping-ponging: {analysis['is_ping_ponging']}")
        print(f"  Max lateral deviation: {analysis['lateral_max']*1000:.2f}mm")
        print(f"  Oscillation frequency: {analysis['oscillation_frequency']:.2f} Hz")
        print(f"  Growing oscillations: {analysis['is_growing']}")

