# Ping-Pong Behavior Analysis

## Overview

The "ping-pong" column in the simulation results indicates whether the robot exhibits oscillatory behavior, bouncing between the two rail flanges. This analysis is based on industry standards for rail-guided vehicles and automated guided vehicles (AGVs).

## Industry Standards

Based on research from rail-guided vehicle systems and automotive industry standards:

1. **Oscillation Frequency**: Oscillation frequency > 0.5 Hz indicates problematic behavior
2. **Rail Hits**: 
   - < 5 hits per 10m travel: Acceptable behavior
   - 5-10 hits per 10m: Marginal, may indicate instability
   - > 10 hits per 10m: Indicates ping-ponging behavior
3. **Amplitude Growth**: Growing oscillation amplitude indicates instability

## Ping-Pong Detection Criteria

The simulation detects ping-ponging if ANY of the following conditions are met:

1. **High Oscillation Frequency**: `oscillation_frequency > threshold`
   - Threshold: 0.5 Hz for spacings ≤ 50mm
   - Threshold: 0.3 Hz for spacings > 50mm (more lenient for larger gaps)

2. **Excessive Rail Hits**: `rail_hits > 10` per 10m travel
   - Each "hit" represents a contact transition (from no contact to contact)
   - Counts both left and right rail contacts

3. **Growing Oscillations**: `is_growing AND y_max > spacing * 1.5`
   - Amplitude trend > 1mm indicates growing oscillations
   - Significant amplitude relative to spacing indicates instability

## Rail Hits Metric

The "Rail Hits" column shows the number of times the robot contacts a rail during the 10-meter travel distance. This is calculated by counting transitions from no contact (force < 10N) to contact (force > 10N) for both left and right rails.

- **0-5 hits**: Stable operation, minimal contact
- **5-10 hits**: Some oscillation, but manageable
- **> 10 hits**: Excessive bouncing, indicates ping-ponging

## Physical Interpretation

Ping-ponging occurs when:
1. The robot has sufficient lateral velocity to reach the opposite rail after bouncing off one rail
2. The spacing is large enough to allow significant lateral movement
3. Damping is insufficient to dissipate the lateral kinetic energy
4. The contact forces create a restoring force that overshoots, causing oscillation

## Mitigation Strategies

Based on industry practices:

1. **Reduce Spacing**: Smaller gaps reduce lateral movement range
2. **Increase Damping**: Higher damping coefficients reduce oscillation amplitude
3. **Optimize Contact Stiffness**: Balance between too stiff (hard impacts) and too soft (excessive penetration)
4. **Guide Wheel Design**: Wider guide wheels or different materials can improve stability

## References

- AGV (Automated Guided Vehicle) stability standards
- Rail-guided vehicle oscillation analysis
- Mass-spring-damper system dynamics
- Impedance control principles for contact stability

