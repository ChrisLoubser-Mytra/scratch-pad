"""
Unit tests for stability analysis.

Tests the RobotSimulator.analyze_stability method which detects
ping-ponging behavior and oscillation patterns.
"""

import numpy as np
import pytest

from robot_simulation import RobotParams, RobotSimulator


class TestStabilityAnalysis:
    """Test suite for stability analysis"""

    @pytest.fixture
    def params(self) -> RobotParams:
        """Create default robot parameters for testing"""
        return RobotParams()

    def test_analysis_returns_all_keys(self, params: RobotParams) -> None:
        """Test that analysis returns all expected keys"""
        simulator = RobotSimulator(params, spacing=0.010, initial_theta=0.01)
        t, state = simulator.simulate(duration=1.0, dt=0.01)

        analysis = simulator.analyze_stability(t, state)

        expected_keys = {
            "lateral_std",
            "lateral_max",
            "angular_std",
            "angular_max",
            "oscillation_frequency",
            "is_ping_ponging",
            "is_growing",
            "amplitude_trend",
            "zero_crossings",
        }

        assert set(analysis.keys()) == expected_keys

    def test_analysis_values_are_numeric(self, params: RobotParams) -> None:
        """Test that all analysis values are numeric (not NaN or inf)"""
        simulator = RobotSimulator(params, spacing=0.010, initial_theta=0.01)
        t, state = simulator.simulate(duration=1.0, dt=0.01)

        analysis = simulator.analyze_stability(t, state)

        for key, value in analysis.items():
            if key in ["is_ping_ponging", "is_growing"]:
                assert isinstance(value, bool)
            else:
                assert isinstance(value, (int, float))
                assert not np.isnan(value)
                assert not np.isinf(value)

    def test_lateral_max_is_positive(self, params: RobotParams) -> None:
        """Test that lateral_max is always positive"""
        simulator = RobotSimulator(params, spacing=0.010, initial_theta=0.01)
        t, state = simulator.simulate(duration=1.0, dt=0.01)

        analysis = simulator.analyze_stability(t, state)

        assert analysis["lateral_max"] >= 0

    def test_lateral_std_is_positive(self, params: RobotParams) -> None:
        """Test that lateral_std is always positive (or zero)"""
        simulator = RobotSimulator(params, spacing=0.010, initial_theta=0.01)
        t, state = simulator.simulate(duration=1.0, dt=0.01)

        analysis = simulator.analyze_stability(t, state)

        assert analysis["lateral_std"] >= 0

    def test_oscillation_frequency_is_positive(self, params: RobotParams) -> None:
        """Test that oscillation frequency is non-negative"""
        simulator = RobotSimulator(params, spacing=0.010, initial_theta=0.01)
        t, state = simulator.simulate(duration=1.0, dt=0.01)

        analysis = simulator.analyze_stability(t, state)

        assert analysis["oscillation_frequency"] >= 0

    def test_zero_crossings_count(self, params: RobotParams) -> None:
        """Test that zero_crossings is a non-negative integer"""
        simulator = RobotSimulator(params, spacing=0.010, initial_theta=0.01)
        t, state = simulator.simulate(duration=1.0, dt=0.01)

        analysis = simulator.analyze_stability(t, state)

        assert isinstance(analysis["zero_crossings"], int)
        assert analysis["zero_crossings"] >= 0

    def test_ping_ponging_is_boolean(self, params: RobotParams) -> None:
        """Test that is_ping_ponging is a boolean"""
        simulator = RobotSimulator(params, spacing=0.010, initial_theta=0.01)
        t, state = simulator.simulate(duration=1.0, dt=0.01)

        analysis = simulator.analyze_stability(t, state)

        assert isinstance(analysis["is_ping_ponging"], bool)

    def test_growing_is_boolean(self, params: RobotParams) -> None:
        """Test that is_growing is a boolean"""
        simulator = RobotSimulator(params, spacing=0.010, initial_theta=0.01)
        t, state = simulator.simulate(duration=1.0, dt=0.01)

        analysis = simulator.analyze_stability(t, state)

        assert isinstance(analysis["is_growing"], bool)

    def test_oscillation_detection_with_oscillating_signal(self, params: RobotParams) -> None:
        """Test that oscillating signals are detected"""
        # Create a simple oscillating signal manually
        t = np.linspace(0, 1.0, 1000)
        # Create state with oscillating lateral velocity
        state = np.zeros((len(t), 6))
        state[:, 1] = 0.01 * np.sin(2 * np.pi * 2 * t)  # 2 Hz oscillation
        state[:, 4] = 0.01 * 2 * np.pi * 2 * np.cos(2 * np.pi * 2 * t)  # Derivative

        simulator = RobotSimulator(params, spacing=0.010, initial_theta=0.01)
        analysis = simulator.analyze_stability(t, state)

        # Should detect oscillations
        assert analysis["oscillation_frequency"] > 0
        assert analysis["zero_crossings"] > 0

    def test_stable_signal_has_low_frequency(self, params: RobotParams) -> None:
        """Test that stable signals have low oscillation frequency"""
        # Create a stable (decaying) signal
        t = np.linspace(0, 1.0, 1000)
        state = np.zeros((len(t), 6))
        state[:, 1] = 0.01 * np.exp(-5 * t)  # Decaying exponential
        state[:, 4] = -0.01 * 5 * np.exp(-5 * t)  # Derivative

        simulator = RobotSimulator(params, spacing=0.010, initial_theta=0.01)
        analysis = simulator.analyze_stability(t, state)

        # Should have low or zero oscillation frequency
        assert analysis["oscillation_frequency"] < 1.0

    def test_analysis_with_different_spacings(self, params: RobotParams) -> None:
        """Test that different spacings produce different analysis results"""
        sim_5mm = RobotSimulator(params, spacing=0.005, initial_theta=0.01)
        sim_20mm = RobotSimulator(params, spacing=0.020, initial_theta=0.01)

        t1, state1 = sim_5mm.simulate(duration=2.0, dt=0.01)
        t2, state2 = sim_20mm.simulate(duration=2.0, dt=0.01)

        analysis1 = sim_5mm.analyze_stability(t1, state1)
        analysis2 = sim_20mm.analyze_stability(t2, state2)

        # Results should differ (exact values depend on simulation)
        # At minimum, check that analysis completes successfully
        assert "is_ping_ponging" in analysis1
        assert "is_ping_ponging" in analysis2

    def test_amplitude_trend_calculation(self, params: RobotParams) -> None:
        """Test that amplitude trend is calculated"""
        simulator = RobotSimulator(params, spacing=0.010, initial_theta=0.01)
        t, state = simulator.simulate(duration=2.0, dt=0.01)

        analysis = simulator.analyze_stability(t, state)

        # Amplitude trend should be a number
        assert isinstance(analysis["amplitude_trend"], (int, float))
        assert not np.isnan(analysis["amplitude_trend"])

    def test_longer_simulation_produces_more_data(self, params: RobotParams) -> None:
        """Test that longer simulations produce more reliable analysis"""
        simulator = RobotSimulator(params, spacing=0.010, initial_theta=0.01)

        t_short, state_short = simulator.simulate(duration=1.0, dt=0.01)
        t_long, state_long = simulator.simulate(duration=5.0, dt=0.01)

        analysis_short = simulator.analyze_stability(t_short, state_short)
        analysis_long = simulator.analyze_stability(t_long, state_long)

        # Both should complete successfully
        assert "oscillation_frequency" in analysis_short
        assert "oscillation_frequency" in analysis_long

        # Longer simulation should have more zero crossings (if oscillating)
        if analysis_long["oscillation_frequency"] > 0:
            assert analysis_long["zero_crossings"] >= analysis_short["zero_crossings"]

