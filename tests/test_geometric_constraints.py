"""
Tests for geometric constraints on robot motion
"""

import pytest
import numpy as np

from robot_simulation import RobotParams, RobotSimulator


class TestGeometricConstraints:
    """Tests that robot motion is constrained by geometry"""

    @pytest.fixture
    def params(self) -> RobotParams:
        """Create default robot parameters"""
        return RobotParams()

    def test_lateral_position_bounded(self, params: RobotParams) -> None:
        """Test that lateral position never exceeds 200mm"""
        spacings = [0.001, 0.005, 0.01, 0.02, 0.05, 0.1]  # 1mm to 100mm
        
        for spacing in spacings:
            simulator = RobotSimulator(params, spacing=spacing, initial_theta=0.01)
            t, state, contact_forces = simulator.simulate(max_distance=10.0, duration=20.0)
            
            y = state[:, 1]  # Lateral position
            max_y = np.max(np.abs(y))
            
            assert max_y < 0.2, (
                f"Lateral position exceeds 200mm for spacing {spacing*1000:.1f}mm. "
                f"Max lateral position: {max_y*1000:.1f}mm"
            )

    def test_angular_rotation_bounded(self, params: RobotParams) -> None:
        """Test that angular rotation is bounded by geometric constraints"""
        spacings = [0.001, 0.005, 0.01, 0.02, 0.05, 0.1]
        
        for spacing in spacings:
            simulator = RobotSimulator(params, spacing=spacing, initial_theta=0.01)
            t, state, contact_forces = simulator.simulate(max_distance=10.0, duration=20.0)
            
            theta = state[:, 2]  # Angular orientation
            max_theta = np.max(np.abs(theta))
            
            # Calculate maximum allowed angle based on geometry
            # The spacing parameter IS the gap when robot is centered
            # To contact a flange, robot must move laterally by spacing
            # Max angle = arctan(spacing / wheel_base)
            max_offset = spacing  # The gap itself is the max offset
            max_allowed_angle = np.arctan(max_offset / params.wheel_base) if max_offset > 0 else 0.0
            
            # Allow some tolerance (20% over) for numerical errors and initial conditions
            assert max_theta <= max_allowed_angle * 1.2, (
                f"Angular rotation exceeds geometric limit for spacing {spacing*1000:.1f}mm. "
                f"Max angle: {np.degrees(max_theta):.2f}deg, "
                f"Max allowed: {np.degrees(max_allowed_angle):.2f}deg"
            )

    def test_small_spacing_drives_straight(self, params: RobotParams) -> None:
        """Test that with 1mm spacing, robot drives straight (low lateral drift)"""
        spacing = 0.001  # 1mm
        simulator = RobotSimulator(params, spacing=spacing, initial_theta=0.01)
        t, state, contact_forces = simulator.simulate(max_distance=10.0, duration=20.0)
        
        y = state[:, 1]  # Lateral position
        theta = state[:, 2]  # Angular orientation
        
        # After 1m of travel, lateral drift should be small (<10mm for 1mm spacing)
        # With 1mm spacing, robot should drive very straight
        x = state[:, 0]  # Forward position
        idx_1m = np.where(x >= 1.0)[0]
        if len(idx_1m) > 0:
            y_at_1m = y[idx_1m[0]]
            assert abs(y_at_1m) < 0.01, (  # Allow up to 10mm drift (realistic for small initial misalignment)
                f"Lateral drift at 1m exceeds 10mm for 1mm spacing. "
                f"Drift: {y_at_1m*1000:.1f}mm"
            )
        
        # Angular orientation should stay small (<3 degrees)
        max_theta_deg = np.max(np.abs(theta)) * 180 / np.pi
        assert max_theta_deg < 3.0, (
            f"Angular rotation exceeds 3 degrees for 1mm spacing. "
            f"Max angle: {max_theta_deg:.2f}deg"
        )
        
        # Standard deviation of lateral position should be small
        y_std = np.std(y)
        assert y_std < 0.003, (
            f"Lateral position standard deviation too high for 1mm spacing. "
            f"Std dev: {y_std*1000:.1f}mm"
        )

    def test_large_spacing_shows_instability(self, params: RobotParams) -> None:
        """Test that with 100mm spacing, robot shows instability"""
        spacing = 0.1  # 100mm
        simulator = RobotSimulator(params, spacing=spacing, initial_theta=0.01)
        t, state, contact_forces = simulator.simulate(max_distance=10.0, duration=20.0)
        
        y = state[:, 1]  # Lateral position
        analysis = simulator.analyze_stability(t, state, contact_forces)
        
        # With 100mm spacing, robot has significant freedom to move
        # Should show ping-ponging or high lateral deviation or many rail hits
        # Max angle with 100mm spacing = arctan(100/1200) = 4.76 degrees (significant)
        max_angle_deg = np.max(np.abs(state[:, 2])) * 180 / np.pi
        assert (analysis["is_ping_ponging"] or 
                analysis["lateral_max"] > 0.05 or 
                analysis.get("rail_hits", 0) > 5 or
                max_angle_deg > 3.0), (
            f"100mm spacing should show instability. "
            f"Ping-ponging: {analysis['is_ping_ponging']}, "
            f"Max lateral: {analysis['lateral_max']*1000:.1f}mm, "
            f"Rail hits: {analysis.get('rail_hits', 0)}, "
            f"Max angle: {max_angle_deg:.2f}deg"
        )

    def test_lateral_position_vs_distance(self, params: RobotParams) -> None:
        """Test that lateral position is tracked vs distance traveled"""
        spacing = 0.01  # 10mm
        simulator = RobotSimulator(params, spacing=spacing, initial_theta=0.01)
        t, state, contact_forces = simulator.simulate(max_distance=10.0, duration=20.0)
        
        x = state[:, 0]  # Forward position (distance)
        y = state[:, 1]  # Lateral position
        
        # Check that after 1m, lateral position is reasonable
        idx_1m = np.where(x >= 1.0)[0]
        if len(idx_1m) > 0:
            y_at_1m = y[idx_1m[0]]
            # Should be within reasonable bounds (not >200mm)
            assert abs(y_at_1m) < 0.2, (
                f"Lateral position at 1m exceeds 200mm. "
                f"Position: {y_at_1m*1000:.1f}mm"
            )
        
        # Check that lateral position is within bounds (may be stable if well-damped)
        # For 10mm spacing, robot may be stable (no oscillation) if damping is good
        # The key constraint is that lateral position stays within reasonable bounds
        max_y = np.max(np.abs(y))
        assert max_y < 0.2, (  # Must be within 200mm
            f"Lateral position exceeds 200mm for 10mm spacing. "
            f"Max: {max_y*1000:.1f}mm"
        )

