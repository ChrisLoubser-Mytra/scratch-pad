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
    rail_flange_height: float = 0.02  # m (20mm vertical flange height)

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
        # These may need adjustment for larger spacings:
        # - Lower stiffness for larger gaps (less rigid contact)
        # - Adjust damping to prevent oscillations
        # - Friction affects energy dissipation
        self.contact_stiffness = 1e6  # N/m (stiff contact) - may need reduction for >50mm spacing
        self.contact_damping = 1000  # N·s/m (damping) - may need increase for larger spacings
        self.friction_coefficient = 0.3  # Friction between guide wheel and rail
        
        # Safety thresholds
        self.max_safe_contact_force = 50000.0  # N (50kN - threshold for rail bending concern)
        self.climbing_force_threshold = 0.4  # Fraction of weight - force that could lift robot
        
    def contact_force(
        self, y: float, vy: float
    ) -> Tuple[float, float, float, float]:
        """
        Calculate contact forces from guide wheels on rail flanges

        Args:
            y: Lateral position (m)
            vy: Lateral velocity (m/s)

        Returns:
            Tuple of (force_left, force_right, penetration_left, penetration_right) in Newtons and meters
        """
        # Rail center is at y=0
        # Left flange at y = -rail_width/2, right flange at y = +rail_width/2
        # Guide wheels are positioned at the edges of the guide wheel width
        # The guide wheel outer edges are at y ± guide_wheel_width/2

        guide_wheel_half_width = self.params.guide_wheel_width / 2
        # Account for angular misalignment - wheels can contact at different positions
        # For large spacings, need to consider the robot's orientation
        left_wheel_edge = y - guide_wheel_half_width
        right_wheel_edge = y + guide_wheel_half_width

        left_flange_pos = -self.rail_width / 2
        right_flange_pos = self.rail_width / 2
        
        # For very large spacings, the robot may need significant offset to contact
        # The rail width = guide_wheel_width + 2*spacing
        # So for 200mm spacing: rail_width = 70mm + 400mm = 470mm
        # Flanges at ±235mm, wheels at y ± 35mm
        # Robot needs to be offset by >200mm to contact

        # Calculate gaps: positive gap means wheel is inside (no contact)
        # Negative gap means wheel has penetrated flange (contact)
        gap_left = left_wheel_edge - left_flange_pos  # Positive if wheel edge is to the right of flange
        gap_right = right_flange_pos - right_wheel_edge  # Positive if wheel edge is to the left of flange

        # Contact forces (only when gap is negative, i.e., wheel penetrates flange)
        force_left = 0.0
        force_right = 0.0
        penetration_left = 0.0
        penetration_right = 0.0

        if gap_left < 0:  # Left wheel edge has penetrated left flange
            penetration_left = -gap_left
            # Limit penetration to flange height (climbing check)
            penetration_left = min(penetration_left, self.params.rail_flange_height)
            
            # Normal force (spring-damper) - adjust stiffness for large penetrations
            # For larger spacings, contact may be less rigid
            effective_stiffness = self.contact_stiffness
            if self.spacing > 0.05:  # >50mm spacing
                # Reduce effective stiffness for larger gaps (less rigid contact)
                effective_stiffness = self.contact_stiffness * (0.05 / self.spacing) ** 0.5
            
            normal_force = effective_stiffness * penetration_left + self.contact_damping * vy
            # Friction force (opposes motion)
            friction_force = (
                self.friction_coefficient * abs(normal_force) * np.sign(vy)
                if abs(vy) > 0.01
                else 0.0
            )
            force_left = normal_force + friction_force

        if gap_right < 0:  # Right wheel edge has penetrated right flange
            penetration_right = -gap_right
            # Limit penetration to flange height (climbing check)
            penetration_right = min(penetration_right, self.params.rail_flange_height)
            
            # Normal force (spring-damper) - adjust stiffness for large penetrations
            effective_stiffness = self.contact_stiffness
            if self.spacing > 0.05:  # >50mm spacing
                # Reduce effective stiffness for larger gaps
                effective_stiffness = self.contact_stiffness * (0.05 / self.spacing) ** 0.5
            
            normal_force = effective_stiffness * penetration_right - self.contact_damping * vy
            # Friction force (opposes motion)
            friction_force = (
                self.friction_coefficient * abs(normal_force) * np.sign(-vy)
                if abs(vy) > 0.01
                else 0.0
            )
            force_right = normal_force + friction_force

        return force_left, force_right, penetration_left, penetration_right
    
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
        force_left, force_right, penetration_left, penetration_right = self.contact_force(y, vy)

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
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Run simulation

        Args:
            duration: Simulation duration (s)
            dt: Time step (s)

        Returns:
            Tuple of (time_array, state_history, contact_forces_history)
            contact_forces_history: [N x 4] array with [force_left, force_right, penetration_left, penetration_right]
        """
        t = np.arange(0, duration, dt)

        # Initial state: slight misalignment
        # For large spacings, add initial lateral offset to ensure contact can occur
        # The robot will drift laterally due to misalignment as it moves forward
        # For large spacings, start at a position where contact will occur
        initial_y_offset = 0.0
        if self.spacing > 0.05:  # >50mm spacing
            # Contact occurs when wheel edge exceeds flange position
            # Right flange at: rail_width/2 = (guide_wheel_width + 2*spacing)/2
            # Right wheel edge at: y + guide_wheel_width/2
            # Contact when: y + guide_wheel_width/2 > rail_width/2
            # So: y > rail_width/2 - guide_wheel_width/2
            # Start just past this threshold to ensure contact
            contact_threshold = self.rail_width / 2 - self.params.guide_wheel_width / 2
            initial_y_offset = contact_threshold + 0.01  # 10mm past threshold to ensure contact
        
        initial_state = np.array([
            0.0,  # x
            initial_y_offset,  # y (with offset for large spacings)
            self.initial_theta,  # theta (small initial misalignment)
            0.0,  # vx
            0.0,  # vy
            0.0,  # omega
        ])

        # Integrate ODE
        solution = odeint(self.dynamics, initial_state, t)

        # Calculate contact forces for each time step
        contact_forces = np.zeros((len(t), 4))
        for i in range(len(t)):
            y = solution[i, 1]
            vy = solution[i, 4]
            force_left, force_right, pen_left, pen_right = self.contact_force(y, vy)
            contact_forces[i] = [force_left, force_right, pen_left, pen_right]

        return t, solution, contact_forces
    
    def analyze_stability(
        self, t: np.ndarray, state: np.ndarray, contact_forces: np.ndarray
    ) -> Dict[str, Any]:
        """
        Analyze simulation results for ping-ponging behavior, climbing risk, and energy imparted

        Args:
            t: Time array
            state: State history [N x 6]
            contact_forces: Contact forces history [N x 4] with [force_left, force_right, pen_left, pen_right]

        Returns:
            Dictionary with analysis results
        """
        y = state[:, 1]  # Lateral position
        theta = state[:, 2]  # Angular orientation
        vy = state[:, 4]  # Lateral velocity
        omega = state[:, 5]  # Angular velocity

        # Extract contact forces
        force_left = contact_forces[:, 0]
        force_right = contact_forces[:, 1]
        penetration_left = contact_forces[:, 2]
        penetration_right = contact_forces[:, 3]

        # Calculate statistics
        y_std = float(np.std(y))
        y_max = float(np.max(np.abs(y)))
        theta_std = float(np.std(theta))
        theta_max = float(np.max(np.abs(theta)))

        # Contact force analysis
        max_contact_force = float(np.max(np.abs(force_left) + np.abs(force_right)))
        max_penetration = float(np.max(np.maximum(penetration_left, penetration_right)))
        
        # Energy imparted to rails (integral of force * velocity)
        # Power = force * velocity, Energy = integral of power
        power_left = force_left * np.abs(vy) * np.sign(force_left)  # Only when moving into contact
        power_right = force_right * np.abs(vy) * np.sign(-force_right)
        total_power = power_left + power_right
        energy_imparted = float(np.trapz(np.maximum(total_power, 0), t))  # Only positive energy (into rails)

        # Climbing risk analysis
        # Check if penetration approaches flange height (20mm)
        climbing_risk = max_penetration / self.params.rail_flange_height if self.params.rail_flange_height > 0 else 0.0
        
        # Check if vertical component of force could lift robot
        # For climbing, need force component that could overcome weight
        total_mass = self.params.robot_mass + self.params.max_pallet_mass
        weight = total_mass * 9.81  # N
        # If contact force is large and at angle, vertical component could lift
        # Simplified: if max force > threshold fraction of weight, climbing risk exists
        max_force_ratio = max_contact_force / weight if weight > 0 else 0.0
        climbing_force_risk = max_force_ratio > self.climbing_force_threshold

        # Detect oscillations (ping-ponging) - improved for larger spacings
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

        # Determine stability - improved detection for larger spacings
        # For larger spacings, use lower frequency threshold (oscillations more likely)
        freq_threshold = 0.3 if self.spacing > 0.05 else 0.5
        # Ping-ponging occurs if: oscillations exist AND amplitude is significant
        is_ping_ponging = (oscillation_frequency > freq_threshold) and (
            y_max > self.spacing * 1.5 or is_growing
        )

        # Safety assessments
        excessive_force = max_contact_force > self.max_safe_contact_force
        high_energy = energy_imparted > 1000.0  # Joules - threshold for concern
        climbing_risk_high = climbing_risk > 0.8 or climbing_force_risk

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
            "max_contact_force": max_contact_force,
            "max_penetration": max_penetration,
            "energy_imparted": energy_imparted,
            "climbing_risk": climbing_risk,
            "climbing_force_risk": climbing_force_risk,
            "excessive_force": excessive_force,
            "high_energy": high_energy,
            "climbing_risk_high": climbing_risk_high,
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

        t, state, contact_forces = simulator.simulate(duration=duration)
        analysis = simulator.analyze_stability(t, state, contact_forces)

        results[spacing_mm] = {
            "time": t,
            "state": state,
            "contact_forces": contact_forces,
            "analysis": analysis,
            "simulator": simulator,
        }

    return results


if __name__ == "__main__":
    # Test simulation
    spacings = [5, 10, 20, 30, 40, 100, 200]  # mm
    results = run_spacing_analysis(spacings, duration=10.0)

    # Print results
    print("Spacing Analysis Results:")
    print("-" * 80)
    for spacing_mm, data in results.items():
        analysis = data["analysis"]
        print(f"\nSpacing: {spacing_mm}mm")
        print(f"  Ping-ponging: {analysis['is_ping_ponging']}")
        print(f"  Max lateral deviation: {analysis['lateral_max']*1000:.2f}mm")
        print(f"  Oscillation frequency: {analysis['oscillation_frequency']:.2f} Hz")
        print(f"  Growing oscillations: {analysis['is_growing']}")
        print(f"  Max contact force: {analysis['max_contact_force']/1000:.2f} kN")
        print(f"  Energy imparted: {analysis['energy_imparted']:.2f} J")
        print(f"  Climbing risk: {analysis['climbing_risk']*100:.1f}%")
        print(f"  Excessive force: {analysis['excessive_force']}")
        print(f"  High energy: {analysis['high_energy']}")
        print(f"  Climbing risk high: {analysis['climbing_risk_high']}")

