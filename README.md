# Mytra Robot Guide Wheel Spacing Analysis

A Python simulation and web-based analysis tool for evaluating the stability of warehouse robots traveling on rails, specifically analyzing the effect of guide wheel spacing on ping-ponging behavior.

## Overview

This simulation models a Mytra warehouse automation robot with the following characteristics:
- **Robot mass**: 500 lbs (227 kg)
- **Max payload**: 3000 lbs (1361 kg)
- **Max speed**: 1.5 m/s
- **Acceleration**: 0.75 m/s²
- **Guide wheels**: 70mm wide, maintaining alignment on rails
- **Movement**: Single-axis motion (X or Y direction)

The simulation answers the critical question: **How much spacing can be allowed between the bot's guide wheels and the vertical rail flanges before the bot starts ping-ponging (oscillating) while trying to drive straight?**

## Features

- **Physics-based simulation** using ODE integration
- **Multiple spacing analysis** (5mm, 10mm, 20mm, 30mm, 40mm, or custom values)
- **Stability detection** to identify ping-ponging behavior
- **Interactive web dashboard** with real-time visualization
- **Comprehensive analysis** including:
  - Lateral position over time
  - Angular orientation tracking
  - Phase plots (position vs velocity)
  - Oscillation frequency analysis
  - Maximum deviation metrics

## Installation

1. Install Python 3.12

2. Install Poetry (if not already installed):
```bash
curl -sSL https://install.python-poetry.org | python3 -
```

3. Install project dependencies:
```bash
poetry install
```

4. Activate the virtual environment:
```bash
poetry shell
```

## Usage

### Web Interface (Recommended)

Launch the interactive web dashboard:

```bash
poetry run python app.py
```

Or if you're in the Poetry shell:

```bash
python app.py
```

Then open your browser to `http://localhost:8050`

The web interface allows you to:
- Configure spacing values to test
- Adjust simulation duration
- Set initial misalignment angle
- View interactive graphs and analysis results
- Compare multiple spacing scenarios

### Command Line

Run the simulation directly:

```bash
poetry run python robot_simulation.py
```

Or if you're in the Poetry shell:

```bash
python robot_simulation.py
```

This will run a test simulation with default spacing values (5, 10, 20, 30, 40 mm) and print results to the console.

## Simulation Model

The simulation models:

1. **Robot Dynamics**: Mass, inertia, and motion constraints
2. **Guide Wheel Contact**: Spring-damper contact forces between guide wheels and rail flanges
3. **Friction**: Frictional forces opposing lateral motion
4. **Coupling Effects**: Interaction between lateral motion and angular orientation

### Key Parameters

- **Contact Stiffness**: 1e6 N/m (stiff contact between wheels and rails)
- **Contact Damping**: 1000 N·s/m (damping to prevent excessive oscillations)
- **Friction Coefficient**: 0.3 (between guide wheels and rail flanges)

### Stability Analysis

The simulation detects ping-ponging behavior by:
- Counting zero crossings in lateral velocity
- Analyzing oscillation frequency
- Tracking amplitude growth over time
- Comparing maximum deviations to spacing tolerance

## Results Interpretation

- **Green bars/curves**: Stable behavior (no ping-ponging)
- **Red bars/curves**: Unstable behavior (ping-ponging detected)
- **Oscillation Frequency**: Higher values indicate more rapid oscillations
- **Max Lateral Deviation**: Should be compared to spacing tolerance

## Customization

You can modify simulation parameters in `robot_simulation.py`:

- `RobotParams`: Physical robot parameters
- `RobotSimulator`: Contact force models and dynamics
- `contact_stiffness`, `contact_damping`, `friction_coefficient`: Contact parameters

## Technical Details

The simulation uses:
- **scipy.integrate.odeint**: For numerical integration of the system dynamics
- **Dash/Plotly**: For interactive web visualization
- **NumPy**: For numerical computations

## License

This is a simulation tool for analysis purposes. Modify as needed for your specific use case.

