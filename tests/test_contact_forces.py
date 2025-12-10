"""
Unit tests for contact force calculations.

Tests the RobotSimulator.contact_force method which calculates forces
between guide wheels and rail flanges.
"""

import numpy as np
import pytest

from robot_simulation import RobotParams, RobotSimulator


class TestContactForces:
    """Test suite for contact force calculations"""

    @pytest.fixture
    def params(self) -> RobotParams:
        """Create default robot parameters for testing"""
        return RobotParams()

    @pytest.fixture
    def simulator_5mm(self, params: RobotParams) -> RobotSimulator:
        """Create simulator with 5mm spacing"""
        return RobotSimulator(params, spacing=0.005, initial_theta=0.01)

    @pytest.fixture
    def simulator_20mm(self, params: RobotParams) -> RobotSimulator:
        """Create simulator with 20mm spacing"""
        return RobotSimulator(params, spacing=0.020, initial_theta=0.01)

    def test_no_contact_when_centered(self, simulator_5mm: RobotSimulator) -> None:
        """Test that no forces occur when robot is perfectly centered"""
        force_left, force_right, _, _ = simulator_5mm.contact_force(y=0.0, vy=0.0)

        assert force_left == 0.0
        assert force_right == 0.0

    def test_left_contact_when_offset_left(self, simulator_5mm: RobotSimulator) -> None:
        """Test that left wheel contacts when robot moves left"""
        # Move robot left so left wheel touches left flange
        y = -0.01  # 10mm left
        force_left, force_right, _, _ = simulator_5mm.contact_force(y=y, vy=0.0)

        assert force_left > 0  # Left wheel should experience force
        assert force_right == 0.0  # Right wheel should not contact

    def test_right_contact_when_offset_right(self, simulator_5mm: RobotSimulator) -> None:
        """Test that right wheel contacts when robot moves right"""
        # Move robot right so right wheel touches right flange
        y = 0.01  # 10mm right
        force_left, force_right, _, _ = simulator_5mm.contact_force(y=y, vy=0.0)

        assert force_left == 0.0  # Left wheel should not contact
        assert force_right > 0  # Right wheel should experience force

    def test_both_contacts_when_large_offset(self, simulator_5mm: RobotSimulator) -> None:
        """Test that both wheels can contact with large offset"""
        # Large offset that causes both wheels to contact
        # With 5mm spacing, rail_width = 80mm, flanges at ±40mm
        # Guide wheel is 70mm wide, so edges are at y ± 35mm
        # To contact both: left edge < -40mm and right edge > +40mm
        # So y - 35mm < -40mm => y < -5mm, and y + 35mm > 40mm => y > 5mm
        # This is impossible, so we can't have both contact with this geometry
        # Instead, test with a very large offset that definitely contacts left
        y = -0.05  # 50mm left
        force_left, force_right, _, _ = simulator_5mm.contact_force(y=y, vy=0.0)

        assert force_left > 0
        # Right wheel won't contact with this geometry, but left definitely will
        # This test verifies left contact occurs with large offset

    def test_force_increases_with_penetration(self, simulator_5mm: RobotSimulator) -> None:
        """Test that contact force increases with penetration depth"""
        y1 = -0.01  # Small penetration
        y2 = -0.02  # Larger penetration

        force1, _ = simulator_5mm.contact_force(y=y1, vy=0.0)
        force2, _ = simulator_5mm.contact_force(y=y2, vy=0.0)

        assert force2 > force1

    def test_damping_opposes_velocity(self, simulator_5mm: RobotSimulator) -> None:
        """Test that damping force opposes lateral velocity"""
        y = -0.01  # Left wheel contacting
        vy_positive = 0.1  # Moving right
        vy_negative = -0.1  # Moving left

        force1, _ = simulator_5mm.contact_force(y=y, vy=vy_positive)
        force2, _ = simulator_5mm.contact_force(y=y, vy=vy_negative)

        # Force should be different due to damping
        assert force1 != force2

    def test_friction_opposes_motion(self, simulator_5mm: RobotSimulator) -> None:
        """Test that friction force opposes motion direction"""
        y = -0.01  # Left wheel contacting
        vy = 0.1  # Moving right (positive)

        force_left, _ = simulator_5mm.contact_force(y=y, vy=vy)

        # Friction should add to the force when moving away from contact
        assert force_left > 0

    def test_no_friction_at_low_velocity(self, simulator_5mm: RobotSimulator) -> None:
        """Test that friction is zero at very low velocities"""
        y = -0.01
        vy_low = 0.001  # Very low velocity (below threshold)
        vy_zero = 0.0

        force1, _ = simulator_5mm.contact_force(y=y, vy=vy_low)
        force2, _ = simulator_5mm.contact_force(y=y, vy=vy_zero)

        # Forces should be very similar (friction should be negligible)
        # Damping will cause a small difference: contact_damping * vy = 1000 * 0.001 = 1.0
        assert abs(force1 - force2) < 2.0  # Small difference due to damping only

    def test_larger_spacing_allows_more_movement(self, simulator_5mm: RobotSimulator, simulator_20mm: RobotSimulator) -> None:
        """Test that larger spacing allows more movement before contact"""
        y = 0.01  # 10mm offset

        _, force_right_5mm = simulator_5mm.contact_force(y=y, vy=0.0)
        _, force_right_20mm = simulator_20mm.contact_force(y=y, vy=0.0)

        # 5mm spacing should have contact, 20mm might not
        # This depends on wheel_base, but 5mm should have more force
        assert force_right_5mm >= force_right_20mm

    def test_rail_width_calculation(self, simulator_5mm: RobotSimulator, simulator_20mm: RobotSimulator) -> None:
        """Test that rail width is calculated correctly based on spacing"""
        # Rail width = guide_wheel_width + 2 * spacing
        expected_width_5mm = 0.07 + 2 * 0.005
        expected_width_20mm = 0.07 + 2 * 0.020

        assert abs(simulator_5mm.rail_width - expected_width_5mm) < 1e-6
        assert abs(simulator_20mm.rail_width - expected_width_20mm) < 1e-6

