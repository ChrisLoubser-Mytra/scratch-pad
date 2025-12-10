# Robot Simulation Architecture

## Overview

The robot simulation has been refactored into a modular structure for better organization and maintainability. The code is now split into logical components that handle specific aspects of the simulation.

## Directory Structure

```
robot/
├── __init__.py          # Package exports
├── params.py            # RobotParams dataclass
├── state.py             # SimulationState dataclass
├── geometry.py          # WheelLayout and RailGeometry classes
├── contact.py           # ContactModel class
├── dynamics.py          # Dynamics class
├── simulator.py         # RobotSimulator class (main simulator)
├── analysis.py          # StabilityAnalyzer class
└── spacing_analysis.py  # run_spacing_analysis function
```

## Component Descriptions

### `params.py` - Robot Parameters
- **RobotParams**: Physical parameters of the robot
  - Mass, wheel dimensions, guide wheel dimensions
  - Rail dimensions (flange height, horizontal width)
  - Calculates moment of inertia

### `state.py` - Simulation State
- **SimulationState**: State vector representation
  - Position (x, y), orientation (theta)
  - Velocities (vx, vy, omega)

### `geometry.py` - Geometry Calculations
- **WheelLayout**: Calculates wheel positions and maximum offset
  - 8 drive wheels: 4 in a line on each side
  - 4 guide wheels: 2 on each side, between drive wheel sets
  - Calculates maximum lateral offset based on wheel spacing
- **RailGeometry**: Rail geometry including straightness/angle
  - Fixed flange separation (48 inches = 1219.2mm)
  - Supports constant rail angle and curvature (angle per meter)
  - Calculates flange positions at any point along the rail

### `contact.py` - Contact Force Model
- **ContactModel**: Models contact between guide wheels and rail flanges
  - Spring-damper contact model
  - Accounts for rail angle/curvature
  - Calculates penetration and forces
  - Includes friction

### `dynamics.py` - Dynamics Equations
- **Dynamics**: Robot dynamics calculations
  - Forward motion (acceleration to max speed)
  - Lateral dynamics from guide wheel contact
  - Angular dynamics with damping
  - Coupling effects

### `simulator.py` - Main Simulator
- **RobotSimulator**: Main simulation class
  - Integrates all components
  - Runs simulation using ODE integration
  - Provides legacy interfaces for backward compatibility
  - Supports rail angle and curvature parameters

### `analysis.py` - Stability Analysis
- **StabilityAnalyzer**: Analyzes simulation results
  - Ping-ponging detection
  - Climbing risk assessment
  - Energy imparted to rails
  - Safety thresholds

### `spacing_analysis.py` - Analysis Functions
- **run_spacing_analysis**: Runs simulation for multiple spacing values
  - Supports rail angle and curvature
  - Converts skew in mm to angular misalignment
  - Returns results dictionary

## Key Features

### Wheel Layout
- **8 Drive Wheels**: 4 wheels in a single line on each side
  - Positioned on rail horizontal surfaces
  - First set: 2 wheels (105mm apart)
  - Gap: 749mm
  - Second set: 2 wheels (105mm apart)
- **4 Guide Wheels**: 2 on each side, between drive wheel sets
  - 464mm apart front-to-back on each side
  - 1119.2mm apart between left and right sides
  - Contact flanges from the inside

### Maximum Offset Calculation
- Based on wheel spacing and rail geometry
- Limited by:
  1. Drive wheels staying on rail horizontal surfaces
  2. Guide wheels staying within flange spacing
- Calculated dynamically based on spacing parameter

### Rail Straightness
- **Constant Angle**: Rails can be angled relative to straight (rad)
- **Curvature**: Rails can have angle change per meter (rad/m)
- Affects flange positions along the rail
- Used in contact force calculations

## Backward Compatibility

The `robot_simulation.py` file provides a compatibility layer that re-exports all classes and functions from the new modular structure. Existing code continues to work without changes.

## Usage Example

```python
from robot import RobotParams, RobotSimulator, run_spacing_analysis
import math

# Create simulator with rail angle
params = RobotParams()
simulator = RobotSimulator(
    params,
    spacing=0.01,  # 10mm spacing
    initial_theta=0.01,
    rail_angle=math.radians(1.0),  # 1 degree constant angle
    rail_angle_per_meter=math.radians(0.1)  # 0.1 deg/m curvature
)

# Run simulation
t, state, contact_forces = simulator.simulate(max_distance=10.0)

# Analyze results
analysis = simulator.analyze_stability(t, state, contact_forces)
```

## Testing

All existing tests continue to work through the compatibility layer. The modular structure makes it easier to test individual components in isolation.

