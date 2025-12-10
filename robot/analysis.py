"""
Stability analysis functions
"""

from typing import Dict, Any
import numpy as np

from robot.params import RobotParams


class StabilityAnalyzer:
    """Analyzes simulation results for stability, ping-ponging, and safety"""
    
    def __init__(self, params: RobotParams, spacing: float) -> None:
        """
        Initialize stability analyzer
        
        Args:
            params: Robot physical parameters
            spacing: Gap between guide wheels and flanges (m)
        """
        self.params = params
        self.spacing = spacing
        self.max_safe_contact_force = 50000.0  # N (50kN)
        self.climbing_force_threshold = 0.4  # Fraction of weight
    
    def analyze(
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
        # Calculate total contact force (sum of left and right, not absolute sum)
        # This represents the peak force experienced by either rail
        total_force = np.abs(force_left) + np.abs(force_right)
        max_contact_force = float(np.max(total_force)) if len(total_force) > 0 else 0.0
        max_penetration = float(np.max(np.maximum(penetration_left, penetration_right)))
        
        # Energy imparted to rails
        # Energy is imparted when force and velocity are in the same direction
        # Left rail: force_left > 0 and vy < 0 (moving left into left rail)
        # Right rail: force_right > 0 and vy > 0 (moving right into right rail)
        # Power = force × velocity (positive when energy is imparted into rail)
        power_left = np.where((force_left > 0) & (vy < 0), force_left * (-vy), 0.0)
        power_right = np.where((force_right > 0) & (vy > 0), force_right * vy, 0.0)
        total_power = power_left + power_right
        # Integrate power over time to get energy
        energy_imparted = float(np.trapz(total_power, t)) if len(total_power) > 0 and len(t) > 1 else 0.0
        
        # Climbing risk analysis
        climbing_risk = max_penetration / self.params.rail_flange_height if self.params.rail_flange_height > 0 else 0.0
        total_mass = self.params.robot_mass + self.params.max_pallet_mass
        weight = total_mass * 9.81  # N
        max_force_ratio = max_contact_force / weight if weight > 0 else 0.0
        climbing_force_risk = max_force_ratio > self.climbing_force_threshold
        
        # Detect oscillations (ping-ponging)
        # Based on industry standards for rail-guided vehicles:
        # - Oscillation frequency > 0.5 Hz indicates problematic behavior
        # - Multiple zero crossings indicate bouncing between rails
        # Count zero crossings in lateral velocity (direction changes)
        # Each zero crossing represents a change from moving left to right or vice versa
        vy_sign_changes = int(np.sum(np.diff(np.sign(vy)) != 0))
        # Frequency = number of oscillations per second
        # Each oscillation = 2 zero crossings (left->right->left)
        oscillation_frequency = float(vy_sign_changes / (2 * t[-1]) if t[-1] > 0 else 0.0)
        
        # Count number of rail contacts (hits) during the simulation
        # A "hit" occurs when contact force transitions from zero to non-zero
        # This represents the number of times the robot contacts a rail during travel
        # Industry standard: < 5 hits per 10m is acceptable, > 10 indicates ping-ponging
        if len(force_left) > 1:
            left_contact_transitions = int(np.sum((force_left[1:] > 10.0) & (force_left[:-1] <= 10.0)))  # Threshold to avoid noise
            right_contact_transitions = int(np.sum((force_right[1:] > 10.0) & (force_right[:-1] <= 10.0)))
            total_rail_hits = left_contact_transitions + right_contact_transitions
        else:
            total_rail_hits = 0
        
        # Check if oscillations are growing
        n_windows = 5
        window_size = len(y) // n_windows
        amplitudes: list[float] = []
        
        for i in range(n_windows):
            start_idx = i * window_size
            end_idx = (i + 1) * window_size if i < n_windows - 1 else len(y)
            window_y = y[start_idx:end_idx]
            amplitudes.append(float(np.max(np.abs(window_y))))
        
        if len(amplitudes) >= 2:
            amplitude_trend = float(np.polyfit(range(len(amplitudes)), amplitudes, 1)[0])
            is_growing = amplitude_trend > 0.001
        else:
            amplitude_trend = 0.0
            is_growing = False
        
        # Determine stability - based on industry standards for rail-guided vehicles
        # Industry standards (from AGV and rail-guided vehicle research):
        # - Oscillation frequency > 0.5 Hz indicates problematic behavior
        # - > 10 rail hits per 10m indicates ping-ponging
        # - Growing amplitude indicates instability
        # For larger spacings, use lower frequency threshold (oscillations more likely)
        freq_threshold = 0.3 if self.spacing > 0.05 else 0.5
        
        # Ping-ponging occurs if:
        # 1. High oscillation frequency (> threshold), OR
        # 2. Many rail hits (> 10 per 10m), OR
        # 3. Growing oscillations with significant amplitude
        hits_per_10m = total_rail_hits  # Already normalized to 10m travel
        is_ping_ponging = (
            (oscillation_frequency > freq_threshold) or
            (hits_per_10m > 10) or
            (is_growing and y_max > self.spacing * 1.5)
        )
        
        # Safety assessments
        excessive_force = max_contact_force > self.max_safe_contact_force
        high_energy = energy_imparted > 1000.0  # Joules
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
            "rail_hits": total_rail_hits,  # Number of times robot hits a rail
            "max_contact_force": max_contact_force,
            "max_penetration": max_penetration,
            "energy_imparted": energy_imparted,
            "climbing_risk": climbing_risk,
            "climbing_force_risk": climbing_force_risk,
            "excessive_force": excessive_force,
            "high_energy": high_energy,
            "climbing_risk_high": climbing_risk_high,
        }

