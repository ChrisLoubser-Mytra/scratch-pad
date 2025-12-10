# Contact Model Parameters and Tuning Guide

## Overview

The simulation uses a spring-damper contact model to simulate interactions between the robot's guide wheels and rail flanges. For larger spacing values (especially 100mm and 200mm), certain parameters may need adjustment to accurately model the physical behavior.

## Key Parameters

### 1. Contact Stiffness (`contact_stiffness`)
- **Current Value**: 1e6 N/m (1 MN/m)
- **Location**: `RobotSimulator.__init__()`
- **Effect**: Controls how rigid the contact is. Higher values = stiffer contact.
- **Tuning for Large Spacings**:
  - For spacings > 50mm, the contact may be less rigid (wheels can deflect more before contact)
  - The code automatically reduces effective stiffness for spacings > 50mm using: `effective_stiffness = contact_stiffness * (0.05 / spacing)^0.5`
  - **Recommendation**: For 100-200mm spacings, consider reducing base stiffness to 5e5 - 7e5 N/m

### 2. Contact Damping (`contact_damping`)
- **Current Value**: 1000 N·s/m
- **Location**: `RobotSimulator.__init__()`
- **Effect**: Damping force opposes velocity, preventing oscillations. Higher values = more damping.
- **Tuning for Large Spacings**:
  - Larger spacings allow more movement before contact, potentially leading to higher velocities
  - **Recommendation**: Increase damping to 1500-2000 N·s/m for spacings > 50mm to prevent excessive oscillations

### 3. Friction Coefficient (`friction_coefficient`)
- **Current Value**: 0.3
- **Location**: `RobotSimulator.__init__()`
- **Effect**: Friction opposes motion and dissipates energy. Higher values = more energy dissipation.
- **Tuning for Large Spacings**:
  - Friction helps prevent ping-ponging by dissipating energy
  - **Recommendation**: May need to increase to 0.4-0.5 for larger spacings to improve stability

### 4. Maximum Safe Contact Force (`max_safe_contact_force`)
- **Current Value**: 50000 N (50 kN)
- **Location**: `RobotSimulator.__init__()`
- **Effect**: Threshold for detecting excessive forces that could bend rails
- **Tuning**: Adjust based on rail material and design specifications

### 5. Climbing Force Threshold (`climbing_force_threshold`)
- **Current Value**: 0.4 (40% of robot weight)
- **Location**: `RobotSimulator.__init__()`
- **Effect**: Fraction of robot weight that, if exceeded by contact force, indicates climbing risk
- **Tuning**: Lower values = more conservative (detects climbing risk earlier)

## Automatic Adjustments

The simulation automatically adjusts contact behavior for larger spacings:

1. **Effective Stiffness Reduction**: For spacings > 50mm, stiffness is reduced to model less rigid contact
2. **Penetration Limiting**: Penetration is limited to rail flange height (20mm) to prevent unrealistic climbing

## Expected Behavior at Different Spacings

### 5-20mm Spacing
- **Behavior**: Tight control, minimal oscillation
- **Parameters**: Default values work well
- **Issues**: Low risk of ping-ponging, climbing, or excessive forces

### 30-50mm Spacing
- **Behavior**: Moderate freedom, some oscillation possible
- **Parameters**: May need slight damping increase
- **Issues**: Watch for growing oscillations

### 50-100mm Spacing
- **Behavior**: Significant movement before contact, higher velocities
- **Parameters**: 
  - Reduce stiffness by 20-30%
  - Increase damping to 1500-2000 N·s/m
  - Consider friction increase to 0.4
- **Issues**: 
  - Ping-ponging becomes more likely
  - Higher contact forces
  - Increased energy imparted to rails

### 100-200mm Spacing
- **Behavior**: Large movements, high impact forces
- **Parameters**:
  - Reduce stiffness to 5e5-7e5 N/m
  - Increase damping to 2000-3000 N·s/m
  - Increase friction to 0.4-0.5
- **Issues**:
  - **Ping-ponging**: Very likely, especially at 200mm
  - **Excessive Forces**: Contact forces may exceed safe limits
  - **Climbing Risk**: High risk of wheels climbing over 20mm flange
  - **Energy**: Significant energy imparted to rails (bending concern)

## Tuning Process

1. **Start with default parameters** and run simulation
2. **Check analysis results**:
   - If ping-ponging occurs but shouldn't: Increase damping
   - If forces are too high: Reduce stiffness or increase spacing
   - If climbing risk is high: Reduce stiffness or increase damping
   - If energy is excessive: Increase damping and friction
3. **Iterate** until behavior matches physical expectations

## Physical Interpretation

- **Stiffness**: How much the wheel/rail system deflects under load
- **Damping**: Energy dissipation in the contact (material damping, structural damping)
- **Friction**: Sliding friction between wheel and rail flange
- **Penetration**: How far the wheel compresses into the flange (limited by 20mm height)

## Notes

- The current model assumes linear spring-damper behavior
- Real systems may have non-linear stiffness (hardening/softening)
- Damping may be velocity-dependent in real systems
- For very large spacings (200mm), the contact model may need significant adjustment or a different approach (e.g., impact model)

