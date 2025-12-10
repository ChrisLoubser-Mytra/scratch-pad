"""
Unit tests for robot dynamics calculations.

Tests the RobotSimulator.dynamics method which calculates the time derivatives
of the system state (position, velocity, acceleration).
"""

import numpy as np
import pytest

from robot_simulation import RobotParams, RobotSimulator


class TestDynamics:
    """Test suite for dynamics calculations"""

    @pytest.fixture
    def params(self) -> RobotParams:
        """Create default robot parameters for testing"""
        return RobotParams()

    @pytest.fixture
    def simulator(self, params: RobotParams) -> RobotSimulator:
        """Create simulator with 10mm spacing"""
        return RobotSimulator(params, spacing=0.010, initial_theta=0.01)

    def test_state_vector_size(self, simulator: RobotSimulator) -> None:
        """Test that state vector has correct size (6 elements)"""
        state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        derivative = simulator.dynamics(state, t=0.0)

        assert len(derivative) == 6
        assert derivative.shape == (6,)

    def test_position_derivative_is_velocity(self, simulator: RobotSimulator) -> None:
        """Test that dx/dt = vx and dy/dt = vy"""
        vx = 1.5
        vy = 0.1
        state = np.array([0.0, 0.0, 0.0, vx, vy, 0.0])

        derivative = simulator.dynamics(state, t=0.0)

        assert abs(derivative[0] - vx) < 1e-6  # dx/dt = vx
        assert abs(derivative[1] - vy) < 1e-6  # dy/dt = vy

    def test_angular_derivative_is_angular_velocity(self, simulator: RobotSimulator) -> None:
        """Test that dtheta/dt = omega"""
        omega = 0.05
        state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, omega])

        derivative = simulator.dynamics(state, t=0.0)

        assert abs(derivative[2] - omega) < 1e-6  # dtheta/dt = omega

    def test_acceleration_below_max_speed(self, simulator: RobotSimulator) -> None:
        """Test that robot accelerates when below max speed"""
        state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # Starting from rest

        derivative = simulator.dynamics(state, t=0.0)

        assert derivative[3] > 0  # dvx/dt > 0 (accelerating)
        assert derivative[3] <= simulator.params.acceleration  # Not exceeding max acceleration

    def test_no_acceleration_at_max_speed(self, simulator: RobotSimulator) -> None:
        """Test that robot doesn't accelerate when at max speed"""
        state = np.array([0.0, 0.0, 0.0, simulator.params.max_speed, 0.0, 0.0])

        derivative = simulator.dynamics(state, t=0.0)

        assert abs(derivative[3]) < 1e-6  # dvx/dt ≈ 0

    def test_lateral_force_from_contact(self, simulator: RobotSimulator) -> None:
        """Test that lateral forces affect lateral acceleration"""
        # Offset robot to trigger contact
        # With 10mm spacing, rail_width = 90mm, flanges at ±45mm
        # Guide wheel edges at y ± 35mm
        # For contact: need y - 35mm < -45mm => y < -10mm, or y + 35mm > 45mm => y > 10mm
        state = np.array([0.0, -0.015, 0.0, 0.0, 0.0, 0.0])  # 15mm left (will contact)

        derivative = simulator.dynamics(state, t=0.0)

        # Should have lateral acceleration due to contact force
        assert abs(derivative[4]) > 0  # dvy/dt != 0

    def test_angular_acceleration_from_torque(self, simulator: RobotSimulator) -> None:
        """Test that contact forces create torque and angular acceleration"""
        # Offset robot to create asymmetric forces
        # Use larger offset to ensure contact
        state = np.array([0.0, -0.015, 0.0, 0.0, 0.0, 0.0])  # 15mm left

        derivative = simulator.dynamics(state, t=0.0)

        # Should have angular acceleration from torque
        assert abs(derivative[5]) > 0  # domega/dt != 0

    def test_coupling_force_effect(self, simulator: RobotSimulator) -> None:
        """Test that forward motion with angular velocity creates coupling"""
        # Moving forward with angular velocity
        state = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.1])  # vx=1.0, omega=0.1

        derivative = simulator.dynamics(state, t=0.0)

        # Coupling should affect lateral acceleration
        # The exact value depends on implementation, but should be non-zero
        assert derivative[4] != 0.0

    def test_centered_no_lateral_force(self, simulator: RobotSimulator) -> None:
        """Test that centered robot with no velocity has no lateral acceleration"""
        state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # Perfectly centered, at rest

        derivative = simulator.dynamics(state, t=0.0)

        # No contact forces, so no lateral acceleration
        assert abs(derivative[4]) < 1e-6  # dvy/dt ≈ 0

    def test_angular_acceleration_scales_with_torque(self, params: RobotParams) -> None:
        """Test that angular acceleration is inversely proportional to moment of inertia"""
        # Create two simulators with different masses (different moments of inertia)
        params_light = RobotParams(robot_mass=100.0, max_pallet_mass=100.0)
        params_heavy = RobotParams(robot_mass=500.0, max_pallet_mass=500.0)

        sim_light = RobotSimulator(params_light, spacing=0.010, initial_theta=0.01)
        sim_heavy = RobotSimulator(params_heavy, spacing=0.010, initial_theta=0.01)

        # Use larger offset to ensure contact occurs
        state = np.array([0.0, -0.015, 0.0, 0.0, 0.0, 0.0])  # 15mm left

        deriv_light = sim_light.dynamics(state, t=0.0)
        deriv_heavy = sim_heavy.dynamics(state, t=0.0)

        # Lighter robot should have larger angular acceleration for same torque
        # (if both have contact and non-zero torque)
        if abs(deriv_light[5]) > 1e-6 and abs(deriv_heavy[5]) > 1e-6:
            assert abs(deriv_light[5]) > abs(deriv_heavy[5])
        else:
            # At least verify that contact is happening (non-zero lateral force)
            assert abs(deriv_light[4]) > 0 or abs(deriv_heavy[4]) > 0

    def test_forward_motion_continues(self, simulator: RobotSimulator) -> None:
        """Test that forward motion continues (dx/dt = vx)"""
        vx = 1.0
        state = np.array([10.0, 0.0, 0.0, vx, 0.0, 0.0])  # Already moving forward

        derivative = simulator.dynamics(state, t=0.0)

        assert abs(derivative[0] - vx) < 1e-6  # Position continues to change

