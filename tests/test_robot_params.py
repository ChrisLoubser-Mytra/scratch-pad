"""
Unit tests for RobotParams class.

Tests the physical parameter initialization and derived parameter calculations.
"""

import pytest

from robot_simulation import RobotParams


class TestRobotParams:
    """Test suite for RobotParams dataclass"""

    def test_default_initialization(self) -> None:
        """Test that RobotParams initializes with default values"""
        params = RobotParams()

        assert params.robot_mass == 227.0  # 500 lbs in kg
        assert params.max_pallet_mass == 1361.0  # 3000 lbs in kg
        assert params.wheel_diameter == 0.3  # 300mm in m
        assert params.guide_wheel_width == 0.07  # 70mm in m
        assert params.max_speed == 1.5  # m/s
        assert params.acceleration == 0.75  # m/s²
        assert params.wheel_base == 0.96  # 48 inches in m

    def test_custom_initialization(self) -> None:
        """Test that RobotParams can be initialized with custom values"""
        params = RobotParams(
            robot_mass=200.0,
            max_pallet_mass=1000.0,
            max_speed=2.0,
            acceleration=1.0,
        )

        assert params.robot_mass == 200.0
        assert params.max_pallet_mass == 1000.0
        assert params.max_speed == 2.0
        assert params.acceleration == 1.0

    def test_moment_of_inertia_calculation(self) -> None:
        """Test that moment of inertia is calculated correctly in __post_init__"""
        params = RobotParams()

        # Moment of inertia should be calculated
        assert params.moment_of_inertia > 0

        # Verify calculation: I = m * (w²) / 12
        total_mass = params.robot_mass + params.max_pallet_mass
        expected_moment = total_mass * (params.wheel_base**2) / 12

        assert abs(params.moment_of_inertia - expected_moment) < 1e-6

    def test_moment_of_inertia_scales_with_mass(self) -> None:
        """Test that moment of inertia scales correctly with mass"""
        params1 = RobotParams(robot_mass=100.0, max_pallet_mass=100.0)
        params2 = RobotParams(robot_mass=200.0, max_pallet_mass=200.0)

        # params2 should have double the moment of inertia
        assert abs(params2.moment_of_inertia - 2 * params1.moment_of_inertia) < 1e-6

    def test_moment_of_inertia_scales_with_wheel_base(self) -> None:
        """Test that moment of inertia scales with wheel base squared"""
        params1 = RobotParams(wheel_base=0.5)
        params2 = RobotParams(wheel_base=1.0)

        # params2 should have 4x the moment of inertia (squared relationship)
        assert abs(params2.moment_of_inertia - 4 * params1.moment_of_inertia) < 1e-6

