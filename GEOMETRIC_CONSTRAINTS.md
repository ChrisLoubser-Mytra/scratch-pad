# Geometric Constraints in Robot Simulation

## Overview

The robot's motion is fundamentally constrained by the geometry of the rails and guide wheels. The flanges physically limit how much the robot can rotate, which is a critical constraint that must be properly modeled in the simulation.

## Key Constraint: Maximum Angular Misalignment

The maximum angular misalignment (rotation) is geometrically limited by the flange contact:

```
max_angle = arctan(spacing / wheel_base)
```

Where:
- `spacing` = gap between guide wheel and flange when robot is **perfectly centered**
- `wheel_base = 1200mm` (distance between front and back wheels)

### Understanding the Spacing Parameter

The "spacing" parameter represents the gap between the guide wheel and the vertical flange when the robot is perfectly centered between two parallel rails. For example:
- If spacing = 10mm, there's a 10mm gap on the left side and 10mm gap on the right side when centered
- This means the rails are effectively 20mm too wide (10mm on each side)
- To contact a flange, the robot must move laterally by the spacing amount

### Example Calculations

For **1mm spacing**:
- Gap on each side when centered: 1mm
- Max offset before contact: 1mm
- Max angle: arctan(1/1200) = **0.048 degrees** (very small, robot drives straight)

For **10mm spacing**:
- Gap on each side when centered: 10mm
- Max offset before contact: 10mm
- Max angle: arctan(10/1200) = **0.48 degrees**

For **100mm spacing**:
- Gap on each side when centered: 100mm
- Max offset before contact: 100mm
- Max angle: arctan(100/1200) = **4.76 degrees** (significant, allows ping-ponging)

## Implementation

### 1. Torque Calculation

The torque from guide wheel contact forces uses the correct lever arm:
- Lever arm = `guide_wheel_separation / 2` (232mm)
- This is the distance from robot center to front/back guide wheels
- **Previous bug**: Used `wheel_base / 2` (600mm) - incorrect!

### 2. Angular Constraint Enforcement

The simulation enforces geometric constraints by:
1. Calculating the maximum allowed angle based on spacing
2. Adding a strong restoring torque when angle exceeds the limit
3. Adding a soft restoring torque when approaching the limit (80% of max)

This prevents the robot from rotating more than physically possible.

### 3. Initial Conditions

For small spacings (1mm):
- Robot starts near center
- Initial offset based on angular misalignment: `wheel_base * tan(theta) * 0.5`
- Allows robot to drive straight (as seen in reality)

For larger spacings:
- Start with small offset (20-30% of max offset)
- Allows room for oscillation while ensuring contact occurs

## Expected Behavior

### 1mm Spacing
- **Expected**: Robot drives straight with minimal lateral drift
- **Max lateral position**: < 10mm
- **Max angle**: < 3 degrees
- **Lateral drift at 1m**: < 10mm

### 100mm Spacing
- **Expected**: Significant instability, ping-ponging
- **Max lateral position**: Can approach ±200mm (but should be constrained)
- **Max angle**: Limited by geometry (but larger than 1mm case)
- **Ping-ponging**: Should be detected

## Lateral Position Bounds

The lateral position should never exceed ±200mm because:
- Flange separation: 1219.2mm
- Guide wheel separation: 1119.2mm
- Maximum possible offset: ~50mm when centered
- With spacing, maximum offset increases, but physical constraints limit to ~200mm

## Tests

The `test_geometric_constraints.py` file includes tests to verify:
1. Lateral position never exceeds 200mm
2. Angular rotation is bounded by geometric constraints
3. Small spacing (1mm) drives straight
4. Large spacing (100mm) shows instability
5. Lateral position vs distance traveled is tracked

## Fixes Applied

1. **Corrected torque lever arm**: Changed from `wheel_base/2` to `guide_wheel_separation/2`
2. **Added angular constraint enforcement**: Prevents rotation beyond geometric limits
3. **Fixed initial conditions**: Small spacings start near center
4. **Added distance-based plots**: Show lateral position and angle vs distance traveled
5. **Added comprehensive tests**: Verify constraints are respected

