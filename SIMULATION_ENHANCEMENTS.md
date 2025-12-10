# Simulation Enhancements Summary

## Overview

The simulation has been enhanced to analyze:
1. **Ping-ponging behavior** (oscillations between rail flanges)
2. **Climbing risk** (wheels climbing over 20mm vertical flanges)
3. **Energy imparted to rails** (potential for rail bending)
4. **Excessive contact forces** (risk of structural damage)

## Key Enhancements

### 1. Enhanced Contact Model

- **Penetration tracking**: Now tracks how far wheels penetrate into flanges (limited to 20mm flange height)
- **Adaptive stiffness**: For spacings > 50mm, contact stiffness is reduced to model less rigid contact
- **Force tracking**: Maximum contact forces are tracked throughout simulation

### 2. New Analysis Metrics

#### Maximum Contact Force
- Tracks peak forces during contact
- Threshold: 50 kN (configurable via `max_safe_contact_force`)
- Indicates risk of rail bending or structural damage

#### Energy Imparted to Rails
- Calculates total energy transferred to rails: `E = ∫ (force × velocity) dt`
- Only positive energy (into rails) is counted
- Threshold: 1000 J (configurable)
- High energy indicates potential for rail fatigue/bending

#### Climbing Risk
- **Penetration-based**: Ratio of max penetration to flange height (20mm)
- **Force-based**: Checks if contact force exceeds 40% of robot weight (could lift robot)
- Risk is "high" if penetration > 80% of flange height OR force ratio > 40%

#### Improved Ping-ponging Detection
- Lower frequency threshold for larger spacings (0.3 Hz for >50mm vs 0.5 Hz for smaller)
- Considers both oscillation frequency and amplitude growth
- More sensitive to growing oscillations

### 3. Initial Conditions for Large Spacings

For spacings > 50mm, the simulation automatically:
- Sets initial lateral offset to ensure contact occurs
- This models the fact that with large gaps, the robot needs significant offset to contact flanges

## Expected Results by Spacing

### 5-20mm: Stable Operation
- **Ping-ponging**: Unlikely
- **Forces**: Low (< 10 kN)
- **Energy**: Minimal
- **Climbing**: No risk

### 30-50mm: Moderate Risk
- **Ping-ponging**: Possible with misalignment
- **Forces**: Moderate (10-30 kN)
- **Energy**: Low to moderate
- **Climbing**: Low risk

### 50-100mm: High Risk Zone
- **Ping-ponging**: Likely, especially with initial misalignment
- **Forces**: High (30-50+ kN) - may exceed safe limits
- **Energy**: High - significant energy transfer to rails
- **Climbing**: Moderate to high risk (penetration approaches 20mm limit)

### 100-200mm: Critical Zone
- **Ping-ponging**: Very likely, especially at 200mm
- **Forces**: Very high (50+ kN) - likely exceeds safe limits
- **Energy**: Very high - risk of rail fatigue/bending
- **Climbing**: High risk - wheels likely to climb over 20mm flanges

## Critical Threshold Analysis

Based on the simulation, the following thresholds are identified:

### 0-30mm: Acceptable
- No significant issues expected
- Forces and energy within safe limits
- No climbing risk

### 30-50mm: Marginal
- Some ping-ponging possible
- Forces approaching limits
- Monitor for growing oscillations

### 50-100mm: Problematic
- **Ping-ponging becomes likely**
- **Forces may exceed 50 kN** (rail bending concern)
- **Energy imparted > 1000 J** (fatigue concern)
- **Climbing risk increases** (penetration > 16mm)

### 100-200mm: Unacceptable
- **Definite ping-ponging at 200mm**
- **Forces exceed safe limits** (structural damage risk)
- **Very high energy** (rail bending/fatigue)
- **High climbing risk** (wheels exceed 20mm flange height)

## Variables to Tune

See `CONTACT_MODEL_PARAMETERS.md` for detailed tuning guide. Key parameters:

1. **contact_stiffness** (default: 1e6 N/m)
   - Reduce to 5e5-7e5 for spacings > 50mm
   - Models less rigid contact at larger gaps

2. **contact_damping** (default: 1000 N·s/m)
   - Increase to 1500-3000 for larger spacings
   - Prevents excessive oscillations

3. **friction_coefficient** (default: 0.3)
   - Increase to 0.4-0.5 for larger spacings
   - Helps dissipate energy and prevent ping-ponging

4. **max_safe_contact_force** (default: 50000 N)
   - Adjust based on rail material/design
   - Current value assumes steel rails

5. **climbing_force_threshold** (default: 0.4)
   - Fraction of robot weight
   - Lower = more conservative (detects risk earlier)

## Recommendations

### For 0-30mm Spacing
- **Current parameters work well**
- No adjustments needed
- Safe operation expected

### For 30-50mm Spacing
- **Monitor for ping-ponging**
- May need slight damping increase
- Consider if forces approach limits

### For 50-100mm Spacing
- **Not recommended without design changes**
- Would require:
  - Higher damping (2000+ N·s/m)
  - Reduced stiffness (5e5 N/m)
  - Possibly taller flanges (> 20mm)
  - Stronger rail design

### For 100-200mm Spacing
- **Not viable with current design**
- Would require:
  - Significant damping (3000+ N·s/m)
  - Much lower stiffness (3e5 N/m)
  - Taller flanges (40-50mm)
  - Reinforced rails
  - Or alternative guidance mechanism

## Web Interface Updates

The web interface now displays:
- Maximum contact force (kN)
- Energy imparted (J)
- Climbing risk (%)
- Issues summary (Excessive Force, High Energy, Climbing Risk)

All metrics are color-coded:
- **Green**: Within safe limits
- **Red**: Exceeds thresholds or indicates problems

## Next Steps

1. **Validate with physical testing** at different spacing values
2. **Calibrate parameters** based on real-world measurements
3. **Consider design modifications** if larger spacings are required:
   - Taller flanges
   - Different guide wheel geometry
   - Active damping systems
   - Alternative guidance mechanisms

