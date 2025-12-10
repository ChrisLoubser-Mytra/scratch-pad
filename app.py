"""
Web application for Mytra Robot Guide Wheel Spacing Analysis

Interactive dashboard to visualize and analyze simulation results.
"""

from typing import Any, Dict, List

import dash
from dash import dcc, html, Input, Output, State
from dash.exceptions import PreventUpdate
import numpy as np
import plotly.graph_objs as go

from robot_simulation import (
    RobotParams,
    RobotSimulator,
    run_spacing_analysis,
    skew_mm_to_theta,
)


# Initialize Dash app
app = dash.Dash(__name__)
app.title = "Mytra Robot Guide Wheel Spacing Analysis"

# Define app layout
app.layout = html.Div([
    html.Div([
        html.H1("Mytra Robot Guide Wheel Spacing Analysis", 
                style={'textAlign': 'center', 'marginBottom': '30px'}),
        
        html.Div(id='parameters-table-container', style={'marginBottom': '30px'}),
        
        html.Div([
            html.H3("Robot and Rails Diagram", style={"marginBottom": "15px"}),
            html.Div([
                html.Div([dcc.Graph(id="robot-diagram-top")], style={"marginBottom": "20px"}),  # Top-down view
                html.Div([dcc.Graph(id="robot-diagram-side")], style={"marginBottom": "20px"}),  # Side view
            ], style={"marginBottom": "30px"}),
        ], style={"marginBottom": "30px"}),
        
        html.Div([
            html.Div([
                html.Label("Spacing Values (mm, comma-separated):", 
                          style={'fontWeight': 'bold', 'marginBottom': '5px'}),
                dcc.Input(
                    id='spacing-input',
                    type='text',
                    value='1,5,10,13,17,20,30,40,100',
                    style={'width': '100%', 'padding': '8px'}
                ),
            ], style={'width': '30%', 'display': 'inline-block', 'marginRight': '20px'}),
            
            html.Div([
                html.Label("Simulation Duration (s):", 
                          style={'fontWeight': 'bold', 'marginBottom': '5px'}),
                dcc.Input(
                    id='duration-input',
                    type='number',
                    value=10.0,
                    min=1.0,
                    max=60.0,
                    step=0.5,
                    style={'width': '100%', 'padding': '8px'}
                ),
            ], style={'width': '20%', 'display': 'inline-block', 'marginRight': '20px'}),
            
            html.Div([
                html.Label("Front-to-Back Skew (mm):", 
                          style={'fontWeight': 'bold', 'marginBottom': '5px'}),
                html.Div([
                    dcc.Input(
                        id='skew-input',
                        type='number',
                        value=10.0,
                        min=0.0,
                        max=50.0,
                        step=0.5,
                        style={'width': '100%', 'padding': '8px'}
                    ),
                    html.Small("Lateral offset between front and back of robot", 
                              style={'color': '#666', 'fontSize': '11px', 'display': 'block', 'marginTop': '2px'})
                ]),
            ], style={'width': '20%', 'display': 'inline-block', 'marginRight': '20px'}),
            
            html.Button('Run Simulation', id='run-button', 
                       style={'width': '20%', 'padding': '10px', 'fontSize': '16px',
                              'backgroundColor': '#4CAF50', 'color': 'white', 
                              'border': 'none', 'borderRadius': '5px', 'cursor': 'pointer'})
        ], style={'marginBottom': '30px', 'padding': '20px', 'backgroundColor': '#f5f5f5', 
                  'borderRadius': '10px'}),
        
        html.Div(id='status-message', style={'marginBottom': '20px', 'fontSize': '14px'}),
        
        dcc.Loading(
            id="loading",
            type="default",
            children=[
                html.Div(id='results-container')
            ]
        )
    ], style={'maxWidth': '1400px', 'margin': '0 auto', 'padding': '20px'})
])


@app.callback(
    [Output("parameters-table-container", "children"),
     Output("robot-diagram-top", "figure"),
     Output("robot-diagram-side", "figure")],
    [Input("run-button", "n_clicks")],
    prevent_initial_call=False,
)
def update_parameters_table(n_clicks: int | None) -> tuple[list[Any], go.Figure, go.Figure]:
    """Create parameters table with realistic material property comparisons"""
    # Get actual parameter values from the simulation
    params = RobotParams()
    
    # Create a simulator instance to get contact model parameters
    # Use default spacing of 0.01m (10mm) for reference
    simulator = RobotSimulator(params, spacing=0.01, initial_theta=0.01)
    
    # Define realistic value ranges based on material properties
    # Values based on: steel racking systems, hard rubber AMR wheels
    parameters_data = [
        {
            "Parameter": "Robot Mass",
            "Current Value": f"{params.robot_mass:.1f} kg ({params.robot_mass*2.20462:.0f} lbs)",
            "Realistic Range": "200-300 kg (440-660 lbs) for AMR",
            "Realistic": "Yes" if 200 <= params.robot_mass <= 300 else "Marginal",
            "Simulation Impact": "CRITICAL: Directly affects dynamics and contact forces",
            "Notes": "Typical AMR base weight"
        },
        {
            "Parameter": "Max Pallet Mass",
            "Current Value": f"{params.max_pallet_mass:.1f} kg ({params.max_pallet_mass*2.20462:.0f} lbs)",
            "Realistic Range": "1000-1500 kg (2200-3300 lbs) for standard pallets",
            "Realistic": "Yes" if 1000 <= params.max_pallet_mass <= 1500 else "Marginal",
            "Simulation Impact": "CRITICAL: Affects total mass, moment of inertia, and climbing forces",
            "Notes": "Standard pallet capacity"
        },
        {
            "Parameter": "Drive Wheel Diameter",
            "Current Value": f"{params.drive_wheel_diameter*1000:.0f} mm",
            "Realistic Range": "80-150 mm for AMR drive wheels",
            "Realistic": "Yes" if 80 <= params.drive_wheel_diameter*1000 <= 150 else "No",
            "Notes": "Used for moment of inertia calculation; not directly in contact model"
        },
        {
            "Parameter": "Drive Wheel Width",
            "Current Value": f"{params.drive_wheel_width*1000:.1f} mm",
            "Realistic Range": "30-50 mm for AMR wheels",
            "Realistic": "Yes" if 30 <= params.drive_wheel_width*1000 <= 50 else "No",
            "Notes": "Not used in simulation; informational only"
        },
        {
            "Parameter": "Wheel Spacing in Set",
            "Current Value": f"{params.wheel_spacing_in_set*1000:.0f} mm",
            "Realistic Range": "80-150 mm typical",
            "Realistic": "Yes" if 80 <= params.wheel_spacing_in_set*1000 <= 150 else "No",
            "Notes": "Not used in simulation; informational only"
        },
        {
            "Parameter": "Wheel Set Separation",
            "Current Value": f"{params.wheel_set_separation*1000:.0f} mm",
            "Realistic Range": "600-900 mm for pallet robots",
            "Realistic": "Yes" if 600 <= params.wheel_set_separation*1000 <= 900 else "No",
            "Notes": "Used to calculate wheel base; affects moment of inertia"
        },
        {
            "Parameter": "Guide Wheel Diameter",
            "Current Value": f"{params.guide_wheel_diameter*1000:.0f} mm",
            "Realistic Range": "60-100 mm for guide wheels",
            "Realistic": "Yes" if 60 <= params.guide_wheel_diameter*1000 <= 100 else "No",
            "Notes": "Not directly in contact model; affects contact area indirectly"
        },
        {
            "Parameter": "Guide Wheel Width",
            "Current Value": f"{params.guide_wheel_width*1000:.0f} mm",
            "Realistic Range": "10-25 mm for guide wheels",
            "Realistic": "Yes" if 10 <= params.guide_wheel_width*1000 <= 25 else "No",
            "Notes": "CRITICAL: Directly determines flange separation and contact forces"
        },
        {
            "Parameter": "Max Speed",
            "Current Value": f"{params.max_speed:.2f} m/s ({params.max_speed*3.28084:.1f} ft/s)",
            "Realistic Range": "1.0-2.0 m/s (3.3-6.6 ft/s) for warehouse AMRs",
            "Realistic": "Yes" if 1.0 <= params.max_speed <= 2.0 else "No",
            "Simulation Impact": "Used: Determines forward motion and coupling forces",
            "Notes": "Typical warehouse robot speed"
        },
        {
            "Parameter": "Acceleration",
            "Current Value": f"{params.acceleration:.2f} m/s²",
            "Realistic Range": "0.5-1.0 m/s² for smooth acceleration",
            "Realistic": "Yes" if 0.5 <= params.acceleration <= 1.0 else "No",
            "Simulation Impact": "Used: Affects forward motion dynamics",
            "Notes": "Conservative acceleration for stability"
        },
        {
            "Parameter": "Wheel Base",
            "Current Value": f"{params.wheel_base*1000:.0f} mm ({params.wheel_base*39.3701:.1f} in)",
            "Realistic Range": "800-1200 mm (31-47 in) for pallet-sized robots",
            "Realistic": "Yes" if 800 <= params.wheel_base*1000 <= 1200 else "No",
            "Simulation Impact": "CRITICAL: Used for torque calculation and moment of inertia",
            "Notes": "Distance between outside faces of front/rear wheels"
        },
        {
            "Parameter": "Rail Flange Height",
            "Current Value": f"{params.rail_flange_height*1000:.0f} mm",
            "Realistic Range": "15-25 mm for steel racking flanges",
            "Realistic": "Yes" if 15 <= params.rail_flange_height*1000 <= 25 else "No",
            "Simulation Impact": "CRITICAL: Limits penetration depth for climbing risk calculation",
            "Notes": "Standard steel racking vertical flange"
        },
        {
            "Parameter": "Contact Stiffness",
            "Current Value": f"{simulator.contact_stiffness/1e6:.1f} MN/m ({simulator.contact_stiffness:.0e} N/m)",
            "Realistic Range": "0.5-5 MN/m for steel-on-rubber contact",
            "Realistic": "Yes" if 0.5e6 <= simulator.contact_stiffness <= 5e6 else "Marginal",
            "Simulation Impact": "CRITICAL: Directly determines contact force magnitude",
            "Notes": "Steel rail + hard rubber wheel contact"
        },
        {
            "Parameter": "Contact Damping",
            "Current Value": f"{simulator.contact_damping:.0f} N·s/m",
            "Realistic Range": "500-2000 N·s/m for rubber damping",
            "Realistic": "Yes" if 500 <= simulator.contact_damping <= 2000 else "Marginal",
            "Simulation Impact": "CRITICAL: Affects oscillation damping and energy dissipation",
            "Notes": "Rubber wheel damping characteristics"
        },
        {
            "Parameter": "Friction Coefficient",
            "Current Value": f"{simulator.friction_coefficient:.2f}",
            "Realistic Range": "0.2-0.5 for hard rubber on steel",
            "Realistic": "Yes" if 0.2 <= simulator.friction_coefficient <= 0.5 else "No",
            "Simulation Impact": "Used: Affects energy dissipation and lateral forces",
            "Notes": "Hard rubber wheel on steel rail"
        },
        {
            "Parameter": "Max Safe Contact Force",
            "Current Value": f"{simulator.max_safe_contact_force/1000:.0f} kN ({simulator.max_safe_contact_force:.0f} N)",
            "Realistic Range": "30-100 kN for steel racking rails",
            "Realistic": "Yes" if 30000 <= simulator.max_safe_contact_force <= 100000 else "Marginal",
            "Simulation Impact": "Used: Threshold for excessive force detection",
            "Notes": "Steel racking structural limit"
        },
        {
            "Parameter": "Climbing Force Threshold",
            "Current Value": f"{simulator.climbing_force_threshold:.1f} × weight",
            "Realistic Range": "0.3-0.5 × weight for stability",
            "Realistic": "Yes" if 0.3 <= simulator.climbing_force_threshold <= 0.5 else "Marginal",
            "Simulation Impact": "Used: Threshold for climbing risk detection",
            "Notes": "Fraction of weight that could lift robot"
        },
    ]
    
    # Create table rows
    table_rows = [
        html.Tr([
            html.Th("Parameter", style={"textAlign": "left", "padding": "8px"}),
            html.Th("Current Value", style={"textAlign": "left", "padding": "8px"}),
            html.Th("Realistic Range", style={"textAlign": "left", "padding": "8px"}),
            html.Th("Realistic?", style={"textAlign": "center", "padding": "8px"}),
            html.Th("Simulation Impact", style={"textAlign": "left", "padding": "8px"}),
            html.Th("Notes", style={"textAlign": "left", "padding": "8px"}),
        ])
    ]
    
    for param in parameters_data:
        realistic_color = (
            "green" if param["Realistic"] == "Yes"
            else "orange" if param["Realistic"] == "Marginal"
            else "red"
        )
        # Determine simulation impact
        impact = param.get("Simulation Impact", param["Notes"])
        impact_color = "red" if "CRITICAL" in impact or "Directly" in impact else "orange" if "affects" in impact.lower() or "Used" in impact else "gray"
        
        table_rows.append(
            html.Tr([
                html.Td(param["Parameter"], style={"padding": "8px", "fontWeight": "bold"}),
                html.Td(param["Current Value"], style={"padding": "8px"}),
                html.Td(param["Realistic Range"], style={"padding": "8px"}),
                html.Td(
                    param["Realistic"],
                    style={"padding": "8px", "color": realistic_color, "fontWeight": "bold", "textAlign": "center"},
                ),
                html.Td(impact, style={"padding": "8px", "fontSize": "12px", "color": impact_color, "fontWeight": "bold" if impact_color != "gray" else "normal"}),
                html.Td(param["Notes"], style={"padding": "8px", "fontSize": "12px", "color": "#666"}),
            ])
        )
    
    table_div = html.Div([
        html.H2("Simulation Parameters", style={"marginBottom": "15px"}),
        html.P(
            "Comparison of simulation parameters with realistic material properties for steel racking systems and hard rubber AMR wheels.",
            style={"marginBottom": "15px", "color": "#666"},
        ),
        html.Table(
            table_rows,
            style={
                "width": "100%",
                "borderCollapse": "collapse",
                "border": "1px solid #ddd",
                "marginBottom": "20px",
                "fontSize": "14px",
            },
        ),
        html.Div([
            html.Span("Legend: ", style={"fontWeight": "bold"}),
            html.Span("Yes", style={"color": "green", "fontWeight": "bold", "marginRight": "15px"}),
            html.Span("Marginal", style={"color": "orange", "fontWeight": "bold", "marginRight": "15px"}),
            html.Span("No", style={"color": "red", "fontWeight": "bold"}),
        ], style={"fontSize": "12px", "marginBottom": "20px"}),
    ], style={"marginBottom": "30px", "padding": "20px", "backgroundColor": "#f9f9f9", "borderRadius": "10px"})
    
    # Create diagrams
    fig_top, fig_side = create_robot_diagram()
    
    return [table_div], fig_top, fig_side


@app.callback(
    [Output("results-container", "children"), Output("status-message", "children")],
    [Input("run-button", "n_clicks")],
    [State("spacing-input", "value"), State("duration-input", "value"), State("skew-input", "value")],
)
def update_results(
    n_clicks: int | None, spacing_str: str, duration: float, initial_skew_mm: float
) -> tuple[Any, Any]:
    """Run simulation and update results"""
    if n_clicks is None:
        raise PreventUpdate

    try:
        # Parse spacing values
        spacings = sorted([float(s.strip()) for s in spacing_str.split(",")])

        # Validate inputs
        if duration <= 0 or duration > 60:
            return [], html.Div(
                "Error: Duration must be between 1 and 60 seconds.",
                style={"color": "red"},
            )

        if initial_skew_mm < 0 or initial_skew_mm > 50:
            return [], html.Div(
                "Error: Front-to-back skew must be between 0 and 50 mm.",
                style={"color": "red"},
            )

        # Run simulation (converts skew_mm to theta internally)
        # Travel exactly 10 meters
        results = run_spacing_analysis(spacings, duration=duration, initial_skew_mm=initial_skew_mm, max_distance=10.0)

        # Create visualizations
        status_msg = html.Div(
            f"Simulation complete! Analyzed {len(spacings)} spacing values.",
            style={"color": "green"},
        )

        return create_results_layout(results, spacings), status_msg

    except Exception as e:
        error_msg = f"Error: {str(e)}"
        return [], html.Div(error_msg, style={"color": "red"})


def create_robot_diagram() -> tuple[go.Figure, go.Figure]:
    """Create top-down and side-view diagrams of the robot and rails with dimensions
    
    Returns:
        Tuple of (top_down_fig, side_view_fig)
    """
    params = RobotParams()
    
    # Robot dimensions from CAD
    drive_wheel_diameter = params.drive_wheel_diameter  # 100mm
    drive_wheel_width = params.drive_wheel_width  # 38.1mm
    wheel_spacing_in_set = params.wheel_spacing_in_set  # 105mm
    wheel_set_separation = params.wheel_set_separation  # 749mm
    wheel_base = params.wheel_base  # 1200mm - distance between outside faces
    
    guide_wheel_diameter = params.guide_wheel_diameter  # 80mm
    guide_wheel_width = params.guide_wheel_width  # 16mm
    guide_wheel_separation = params.guide_wheel_separation  # 464mm (on same side)
    guide_wheel_separation_across = params.guide_wheel_separation_across  # 1119.2mm (between left and right)
    
    robot_body_height = params.robot_body_height  # 192mm
    robot_body_clearance = params.robot_body_clearance  # 20mm
    
    # Rail dimensions
    rail_horizontal_width = params.rail_horizontal_width  # 66.04mm (2.6 inches)
    rail_flange_height = params.rail_flange_height  # 19.05mm (0.75 inches)
    flange_separation = params.flange_separation_fixed  # 1219.2mm (48 inches, fixed)
    
    # Example spacing for diagram (use 10mm as example)
    example_spacing = 0.01  # 10mm gap between guide wheel and flange
    
    # Calculate positions
    front_set_center = -wheel_base/2
    rear_set_center = wheel_base/2
    guide_wheel_center_x = (front_set_center + rear_set_center) / 2  # Center between sets
    
    # Guide wheel positions (fixed, not centered)
    # Guide wheels are 1119.2mm apart (between left and right sides)
    guide_wheel_left_y = -guide_wheel_separation_across / 2  # Left guide wheel lateral position
    guide_wheel_right_y = guide_wheel_separation_across / 2  # Right guide wheel lateral position
    
    # Guide wheels on each side are 464mm apart (front to back)
    guide_wheel_left_front_x = guide_wheel_center_x - guide_wheel_separation/2
    guide_wheel_left_rear_x = guide_wheel_center_x + guide_wheel_separation/2
    guide_wheel_right_front_x = guide_wheel_center_x - guide_wheel_separation/2
    guide_wheel_right_rear_x = guide_wheel_center_x + guide_wheel_separation/2
    
    # ========== TOP-DOWN VIEW ==========
    fig_top = go.Figure()
    
    # Robot body (rectangular prism top view)
    robot_body_length = wheel_base * 0.8
    robot_body_width = 0.3
    body_x = [-robot_body_length/2, robot_body_length/2, robot_body_length/2, -robot_body_length/2, -robot_body_length/2]
    body_y = [-robot_body_width/2, -robot_body_width/2, robot_body_width/2, robot_body_width/2, -robot_body_width/2]
    fig_top.add_trace(go.Scatter(x=body_x, y=body_y, fill="toself", fillcolor="lightblue",
                                line=dict(color="blue", width=2), mode="lines", showlegend=False, hoverinfo="skip"))
    
    # Drive wheels as rectangles (top-down view)
    # Left side front set (2 wheels)
    wheel1_x = [front_set_center - drive_wheel_width/2, front_set_center + drive_wheel_width/2,
                front_set_center + drive_wheel_width/2, front_set_center - drive_wheel_width/2, front_set_center - drive_wheel_width/2]
    wheel1_y = [-0.15 - drive_wheel_diameter/2, -0.15 - drive_wheel_diameter/2,
                -0.15 + drive_wheel_diameter/2, -0.15 + drive_wheel_diameter/2, -0.15 - drive_wheel_diameter/2]
    fig_top.add_trace(go.Scatter(x=wheel1_x, y=wheel1_y, fill="toself", fillcolor="darkgray",
                                line=dict(color="black", width=1), mode="lines", showlegend=False, hoverinfo="skip"))
    wheel2_x = [front_set_center + wheel_spacing_in_set - drive_wheel_width/2, front_set_center + wheel_spacing_in_set + drive_wheel_width/2,
                front_set_center + wheel_spacing_in_set + drive_wheel_width/2, front_set_center + wheel_spacing_in_set - drive_wheel_width/2,
                front_set_center + wheel_spacing_in_set - drive_wheel_width/2]
    wheel2_y = wheel1_y
    fig_top.add_trace(go.Scatter(x=wheel2_x, y=wheel2_y, fill="toself", fillcolor="darkgray",
                                line=dict(color="black", width=1), mode="lines", showlegend=False, hoverinfo="skip"))
    # Left side rear set
    wheel3_x = [x + wheel_set_separation for x in wheel1_x]
    wheel3_y = wheel1_y
    fig_top.add_trace(go.Scatter(x=wheel3_x, y=wheel3_y, fill="toself", fillcolor="darkgray",
                                line=dict(color="black", width=1), mode="lines", showlegend=False, hoverinfo="skip"))
    wheel4_x = [x + wheel_set_separation for x in wheel2_x]
    wheel4_y = wheel1_y
    fig_top.add_trace(go.Scatter(x=wheel4_x, y=wheel4_y, fill="toself", fillcolor="darkgray",
                                line=dict(color="black", width=1), mode="lines", showlegend=False, hoverinfo="skip"))
    # Right side wheels (mirror)
    for wheel_x_list, wheel_y_list in [(wheel1_x, wheel1_y), (wheel2_x, wheel2_y), (wheel3_x, wheel3_y), (wheel4_x, wheel4_y)]:
        wheel_y_mirror = [-y for y in wheel_y_list]
        fig_top.add_trace(go.Scatter(x=wheel_x_list, y=wheel_y_mirror, fill="toself", fillcolor="darkgray",
                                    line=dict(color="black", width=1), mode="lines", showlegend=False, hoverinfo="skip"))
    
    # Guide wheels as circles (top-down view)
    # Guide wheels are at fixed positions: left at -559.6mm, right at +559.6mm
    guide_wheel_radius = guide_wheel_diameter / 2
    angles = np.linspace(0, 2*np.pi, 20)
    
    # Left side guide wheels (2 wheels, 464mm apart front to back)
    for gw_x in [guide_wheel_left_front_x, guide_wheel_left_rear_x]:
        wheel_x = gw_x + guide_wheel_radius * np.cos(angles)
        wheel_y = guide_wheel_left_y + guide_wheel_radius * np.sin(angles)
        fig_top.add_trace(go.Scatter(x=wheel_x, y=wheel_y, fill="toself", fillcolor="orange",
                                    line=dict(color="darkorange", width=2), mode="lines", showlegend=False, hoverinfo="skip"))
    
    # Right side guide wheels (2 wheels, 464mm apart front to back)
    for gw_x in [guide_wheel_right_front_x, guide_wheel_right_rear_x]:
        wheel_x = gw_x + guide_wheel_radius * np.cos(angles)
        wheel_y = guide_wheel_right_y + guide_wheel_radius * np.sin(angles)
        fig_top.add_trace(go.Scatter(x=wheel_x, y=wheel_y, fill="toself", fillcolor="orange",
                                    line=dict(color="darkorange", width=2), mode="lines", showlegend=False, hoverinfo="skip"))
    
    # L-shaped rails (top-down view)
    # Horizontal surfaces face toward each other (L's face inward)
    rail_length = wheel_base * 1.3
    
    # Left rail: horizontal surface extends from flange inward
    # Flange is at y = -flange_separation/2 = -609.6mm
    # Horizontal surface extends from flange inward (toward center)
    left_rail_horizontal_x = [-rail_length/2, rail_length/2, rail_length/2, -rail_length/2, -rail_length/2]
    left_rail_horizontal_y = [-flange_separation/2 - rail_horizontal_width, -flange_separation/2 - rail_horizontal_width,
                              -flange_separation/2, -flange_separation/2, -flange_separation/2 - rail_horizontal_width]
    fig_top.add_trace(go.Scatter(x=left_rail_horizontal_x, y=left_rail_horizontal_y, fill="toself", fillcolor="gray",
                                line=dict(color="black", width=2), mode="lines", showlegend=False, hoverinfo="skip"))
    
    # Right rail: horizontal surface extends from flange inward
    right_rail_horizontal_x = left_rail_horizontal_x
    right_rail_horizontal_y = [flange_separation/2, flange_separation/2, flange_separation/2 + rail_horizontal_width,
                                flange_separation/2 + rail_horizontal_width, flange_separation/2]
    fig_top.add_trace(go.Scatter(x=right_rail_horizontal_x, y=right_rail_horizontal_y, fill="toself", fillcolor="gray",
                                line=dict(color="black", width=2), mode="lines", showlegend=False, hoverinfo="skip"))
    
    # Vertical flanges (shown as thick lines in top view)
    flange_thickness = 0.003  # Visual thickness in diagram
    left_flange_x = [-rail_length/2, rail_length/2, rail_length/2, -rail_length/2, -rail_length/2]
    left_flange_y = [-flange_separation/2 - flange_thickness, -flange_separation/2 - flange_thickness,
                     -flange_separation/2, -flange_separation/2, -flange_separation/2 - flange_thickness]
    fig_top.add_trace(go.Scatter(x=left_flange_x, y=left_flange_y, fill="toself", fillcolor="darkgray",
                                line=dict(color="black", width=3), mode="lines", showlegend=False, hoverinfo="skip"))
    right_flange_x = left_flange_x
    right_flange_y = [flange_separation/2, flange_separation/2, flange_separation/2 + flange_thickness,
                      flange_separation/2 + flange_thickness, flange_separation/2]
    fig_top.add_trace(go.Scatter(x=right_flange_x, y=right_flange_y, fill="toself", fillcolor="darkgray",
                                line=dict(color="black", width=3), mode="lines", showlegend=False, hoverinfo="skip"))
    
    # Add dimension annotations for top view
    # Gap between guide wheel and flange
    gap_y_pos = guide_wheel_left_y + (guide_wheel_left_y - (-flange_separation/2)) / 2
    fig_top.add_annotation(x=robot_body_length/2 + 0.1, y=gap_y_pos,
                          text=f"Gap: {example_spacing*1000:.0f}mm", showarrow=True, arrowhead=2, arrowsize=1.5,
                          arrowwidth=2, arrowcolor="red", ax=30, ay=0, font=dict(size=12, color="red"),
                          bgcolor="white", bordercolor="red", borderwidth=2)
    
    # Wheel base
    fig_top.add_annotation(x=0, y=-0.7, text=f"Wheel Base: {wheel_base*1000:.0f}mm", showarrow=True,
                          arrowhead=2, arrowsize=1.5, arrowwidth=2, arrowcolor="blue", ax=0, ay=25,
                          font=dict(size=12, color="blue"), bgcolor="white", bordercolor="blue", borderwidth=2)
    
    # Guide wheel separation (front to back on same side)
    fig_top.add_annotation(x=guide_wheel_center_x, y=guide_wheel_left_y - 0.1,
                          text=f"Guide Wheel Separation: {guide_wheel_separation*1000:.0f}mm<br>(front to back on side)", showarrow=True,
                          arrowhead=2, arrowsize=1, arrowwidth=1.5, arrowcolor="orange", ax=0, ay=20,
                          font=dict(size=11, color="orange"), bgcolor="white", bordercolor="orange", borderwidth=1)
    
    # Guide wheel separation across (left to right)
    fig_top.add_annotation(x=0, y=0,
                          text=f"Guide Wheel Separation: {guide_wheel_separation_across*1000:.1f}mm<br>(between left and right)", showarrow=True,
                          arrowhead=2, arrowsize=1, arrowwidth=1.5, arrowcolor="purple", ax=0, ay=0,
                          font=dict(size=11, color="purple"), bgcolor="white", bordercolor="purple", borderwidth=1)
    
    # Flange separation
    fig_top.add_annotation(x=-robot_body_length/2 - 0.15, y=0,
                          text=f"Flange Separation: {flange_separation*1000:.1f}mm<br>(48 inches, fixed)", showarrow=True,
                          arrowhead=2, arrowsize=1.5, arrowwidth=2, arrowcolor="navy", ax=40, ay=0,
                          font=dict(size=12, color="navy"), bgcolor="white", bordercolor="navy", borderwidth=2)
    
    # Rail horizontal width
    fig_top.add_annotation(x=robot_body_length/2 + 0.1, y=-flange_separation/2 - rail_horizontal_width/2,
                          text=f"Rail Width: {rail_horizontal_width*1000:.1f}mm<br>(2.6 inches)", showarrow=True,
                          arrowhead=2, arrowsize=1, arrowwidth=1.5, arrowcolor="brown", ax=30, ay=0,
                          font=dict(size=11, color="brown"), bgcolor="white", bordercolor="brown", borderwidth=1)
    
    fig_top.update_layout(
        title="Top-Down View: Robot and Rails<br>Robot travels left/right between L-shaped rails",
        xaxis=dict(title="Direction of Travel (m)", range=[-1.0, 1.0], showgrid=True, gridcolor="lightgray"),
        yaxis=dict(title="Lateral Position (m)", range=[-0.7, 0.7], showgrid=True, gridcolor="lightgray"),
        height=600, width=1000, template="plotly_white", showlegend=False, plot_bgcolor="white"
    )
    
    # ========== SIDE VIEW ==========
    fig_side = go.Figure()
    
    # Ground
    ground_y = 0
    
    # L-shaped rails (side view)
    # Left rail: horizontal surface + vertical flange
    rail_horizontal_thickness = 0.01  # Visual thickness
    left_rail_horizontal_x = [-flange_separation/2 - rail_horizontal_width, -flange_separation/2,
                              -flange_separation/2, -flange_separation/2 - rail_horizontal_width,
                              -flange_separation/2 - rail_horizontal_width]
    left_rail_horizontal_y = [ground_y, ground_y, ground_y + rail_horizontal_thickness,
                              ground_y + rail_horizontal_thickness, ground_y]
    fig_side.add_trace(go.Scatter(x=left_rail_horizontal_x, y=left_rail_horizontal_y, fill="toself", fillcolor="gray",
                                  line=dict(color="black", width=2), mode="lines", showlegend=False, hoverinfo="skip"))
    
    # Left rail vertical flange
    left_flange_x = [-flange_separation/2 - 0.005, -flange_separation/2, -flange_separation/2,
                     -flange_separation/2 - 0.005, -flange_separation/2 - 0.005]
    left_flange_y = [ground_y, ground_y, ground_y + rail_flange_height,
                     ground_y + rail_flange_height, ground_y]
    fig_side.add_trace(go.Scatter(x=left_flange_x, y=left_flange_y, fill="toself", fillcolor="darkgray",
                                  line=dict(color="black", width=2), mode="lines", showlegend=False, hoverinfo="skip"))
    
    # Right rail: horizontal surface + vertical flange
    right_rail_horizontal_x = [flange_separation/2, flange_separation/2 + rail_horizontal_width,
                               flange_separation/2 + rail_horizontal_width, flange_separation/2, flange_separation/2]
    right_rail_horizontal_y = left_rail_horizontal_y
    fig_side.add_trace(go.Scatter(x=right_rail_horizontal_x, y=right_rail_horizontal_y, fill="toself", fillcolor="gray",
                                  line=dict(color="black", width=2), mode="lines", showlegend=False, hoverinfo="skip"))
    
    # Right rail vertical flange
    right_flange_x = [flange_separation/2, flange_separation/2 + 0.005, flange_separation/2 + 0.005,
                      flange_separation/2, flange_separation/2]
    right_flange_y = left_flange_y
    fig_side.add_trace(go.Scatter(x=right_flange_x, y=right_flange_y, fill="toself", fillcolor="darkgray",
                                  line=dict(color="black", width=2), mode="lines", showlegend=False, hoverinfo="skip"))
    
    # Drive wheels (circles in side view) - run on horizontal surfaces
    wheel_bottom = ground_y + rail_horizontal_thickness
    wheel_center_y = wheel_bottom + drive_wheel_diameter/2
    wheel_center_x = 0
    wheel_angles = np.linspace(0, 2*np.pi, 30)
    wheel_x_coords = wheel_center_x + (drive_wheel_diameter/2) * np.cos(wheel_angles)
    wheel_y_coords = wheel_center_y + (drive_wheel_diameter/2) * np.sin(wheel_angles)
    fig_side.add_trace(go.Scatter(x=wheel_x_coords, y=wheel_y_coords, fill="toself", fillcolor="darkgray",
                                  line=dict(color="black", width=2), mode="lines", showlegend=False, hoverinfo="skip"))
    
    # Guide wheels (horizontal, contact flanges from inside)
    guide_wheel_center_y = wheel_bottom + drive_wheel_diameter + robot_body_clearance + guide_wheel_diameter/2
    # Left guide wheel contacts left flange from the right side (inside)
    guide_wheel_left_x_coords = guide_wheel_left_y + (guide_wheel_diameter/2) * np.cos(wheel_angles)
    guide_wheel_left_y_coords = guide_wheel_center_y + (guide_wheel_diameter/2) * np.sin(wheel_angles)
    fig_side.add_trace(go.Scatter(x=guide_wheel_left_x_coords, y=guide_wheel_left_y_coords, fill="toself", fillcolor="orange",
                                  line=dict(color="darkorange", width=2), mode="lines", showlegend=False, hoverinfo="skip"))
    # Right guide wheel contacts right flange from the left side (inside)
    guide_wheel_right_x_coords = guide_wheel_right_y + (guide_wheel_diameter/2) * np.cos(wheel_angles)
    guide_wheel_right_y_coords = guide_wheel_left_y_coords
    fig_side.add_trace(go.Scatter(x=guide_wheel_right_x_coords, y=guide_wheel_right_y_coords, fill="toself", fillcolor="orange",
                                  line=dict(color="darkorange", width=2), mode="lines", showlegend=False, hoverinfo="skip"))
    
    # Robot body (rectangular prism)
    body_bottom = wheel_bottom + drive_wheel_diameter + robot_body_clearance
    body_top = body_bottom + robot_body_height
    body_left = -0.15
    body_right = 0.15
    body_x_coords = [body_left, body_right, body_right, body_left, body_left]
    body_y_coords = [body_bottom, body_bottom, body_top, body_top, body_bottom]
    fig_side.add_trace(go.Scatter(x=body_x_coords, y=body_y_coords, fill="toself", fillcolor="lightblue",
                                  line=dict(color="blue", width=2), mode="lines", showlegend=False, hoverinfo="skip"))
    
    # Add dimension annotations for side view
    fig_side.add_annotation(x=0.2, y=body_bottom + robot_body_height/2, text=f"Robot Body: {robot_body_height*1000:.0f}mm tall",
                           showarrow=True, arrowhead=2, arrowsize=1, arrowwidth=1.5, arrowcolor="blue", ax=30, ay=0,
                           font=dict(size=11, color="blue"), bgcolor="white", bordercolor="blue", borderwidth=1)
    fig_side.add_annotation(x=0.2, y=wheel_bottom + drive_wheel_diameter/2, text=f"Drive Wheel: Ø{drive_wheel_diameter*1000:.0f}mm",
                           showarrow=False, font=dict(size=11, color="black"), bgcolor="white", bordercolor="black", borderwidth=1)
    fig_side.add_annotation(x=-0.25, y=guide_wheel_center_y,
                           text=f"Guide Wheel: Ø{guide_wheel_diameter*1000:.0f}mm<br>Clearance: {robot_body_clearance*1000:.0f}mm",
                           showarrow=False, font=dict(size=11, color="darkorange"), bgcolor="white",
                           bordercolor="darkorange", borderwidth=1)
    fig_side.add_annotation(x=-flange_separation/2 - 0.05, y=rail_flange_height/2, text=f"Flange Height: {rail_flange_height*1000:.2f}mm<br>(0.75 inches)",
                           showarrow=True, arrowhead=2, arrowsize=1, arrowwidth=1.5, arrowcolor="red", ax=20, ay=0,
                           font=dict(size=11, color="red"), bgcolor="white", bordercolor="red", borderwidth=1)
    
    fig_side.add_annotation(x=-flange_separation/2 - rail_horizontal_width/2, y=ground_y + rail_horizontal_thickness/2,
                           text=f"Rail Width: {rail_horizontal_width*1000:.1f}mm<br>(2.6 inches)", showarrow=True,
                           arrowhead=2, arrowsize=1, arrowwidth=1.5, arrowcolor="brown", ax=-20, ay=0,
                           font=dict(size=11, color="brown"), bgcolor="white", bordercolor="brown", borderwidth=1)
    
    fig_side.add_annotation(x=0, y=guide_wheel_center_y,
                           text=f"Guide Wheel Separation: {guide_wheel_separation_across*1000:.1f}mm", showarrow=True,
                           arrowhead=2, arrowsize=1, arrowwidth=1.5, arrowcolor="purple", ax=0, ay=0,
                           font=dict(size=11, color="purple"), bgcolor="white", bordercolor="purple", borderwidth=1)
    
    fig_side.update_layout(
        title="Side View: Robot and Rails<br>Showing L-shaped rails with vertical flanges",
        xaxis=dict(title="Lateral Position (m)", range=[-0.7, 0.7], showgrid=True, gridcolor="lightgray"),
        yaxis=dict(title="Height (m)", range=[-0.05, 0.35], showgrid=True, gridcolor="lightgray"),
        height=600, width=800, template="plotly_white", showlegend=False, plot_bgcolor="white"
    )
    
    return fig_top, fig_side


def create_results_layout(
    results: Dict[float, Dict[str, Any]], spacings: List[float]
) -> html.Div:
    """Create the results visualization layout"""
    # Summary table
    summary_data: List[Dict[str, Any]] = []
    for spacing_mm in spacings:
        analysis = results[spacing_mm]["analysis"]
        summary_data.append({
            "Spacing (mm)": spacing_mm,
            "Ping-ponging": "Yes" if analysis["is_ping_ponging"] else "No",
            "Max Lateral Dev (mm)": f"{analysis['lateral_max']*1000:.2f}",
            "Lateral Std Dev (mm)": f"{analysis['lateral_std']*1000:.2f}",
            "Oscillation Freq (Hz)": f"{analysis['oscillation_frequency']:.2f}",
            "Growing": "Yes" if analysis["is_growing"] else "No",
            "Max Force (kN)": f"{analysis['max_contact_force']/1000:.2f}",
            "Energy (J)": f"{analysis['energy_imparted']:.1f}",
            "Climb Risk": f"{analysis['climbing_risk']*100:.0f}%",
            "Issues": ", ".join([
                "Excessive Force" if analysis["excessive_force"] else "",
                "High Energy" if analysis["high_energy"] else "",
                "Climbing Risk" if analysis["climbing_risk_high"] else "",
            ]).strip(", "),
        })
    
    # 1. Lateral position over time (all spacings)
    fig1 = go.Figure()
    # Color palette (replaces plotly.express colors)
    colors = [
        "#1f77b4",  # blue
        "#ff7f0e",  # orange
        "#2ca02c",  # green
        "#d62728",  # red
        "#9467bd",  # purple
        "#8c564b",  # brown
        "#e377c2",  # pink
        "#7f7f7f",  # gray
        "#bcbd22",  # olive
        "#17becf",  # cyan
    ]
    for i, spacing_mm in enumerate(spacings):
        t = results[spacing_mm]["time"]
        y = results[spacing_mm]["state"][:, 1] * 1000  # Convert to mm
        # Clamp y values to reasonable range (should be within ±flange_separation/2)
        # Get flange separation from simulator
        simulator = results[spacing_mm]["simulator"]
        max_y = (simulator.flange_separation / 2 + simulator.params.guide_wheel_width / 2) * 1000  # mm
        min_y = -max_y
        y = np.clip(y, min_y, max_y)  # Clamp to reasonable bounds
        analysis = results[spacing_mm]["analysis"]
        color = "red" if analysis["is_ping_ponging"] else colors[i % len(colors)]

        fig1.add_trace(
            go.Scatter(
                x=t,
                y=y,
                mode="lines",
                name=f"{spacing_mm}mm",
                line=dict(color=color, width=2),
                hovertemplate=f"Spacing: {spacing_mm}mm<br>Time: %{{x:.2f}}s<br>Lateral: %{{y:.2f}}mm<extra></extra>",
            )
        )

    fig1.update_layout(
        title="Lateral Position Over Time",
        xaxis_title="Time (s)",
        yaxis_title="Lateral Position (mm)",
        hovermode="closest",
        height=400,
        template="plotly_white",
    )
    
    # 2. Angular orientation over time
    fig2 = go.Figure()
    for i, spacing_mm in enumerate(spacings):
        t = results[spacing_mm]["time"]
        theta_rad = results[spacing_mm]["state"][:, 2]
        # Convert to degrees and wrap to [-180, 180] for readability
        theta_deg = np.degrees(theta_rad)
        # Wrap to reasonable range (keep within ±180 degrees for display)
        theta_deg = np.mod(theta_deg + 180, 360) - 180
        analysis = results[spacing_mm]["analysis"]
        color = "red" if analysis["is_ping_ponging"] else colors[i % len(colors)]

        fig2.add_trace(
            go.Scatter(
                x=t,
                y=theta_deg,
                mode="lines",
                name=f"{spacing_mm}mm",
                line=dict(color=color, width=2),
                hovertemplate=f"Spacing: {spacing_mm}mm<br>Time: %{{x:.2f}}s<br>Angle: %{{y:.3f}}°<extra></extra>",
            )
        )

    fig2.update_layout(
        title="Angular Orientation Over Time",
        xaxis_title="Time (s)",
        yaxis_title="Angle (degrees)",
        hovermode="closest",
        height=400,
        template="plotly_white",
    )
    
    # 3. Phase plot (lateral position vs lateral velocity)
    fig3 = go.Figure()
    for i, spacing_mm in enumerate(spacings):
        y = results[spacing_mm]["state"][:, 1] * 1000  # mm
        vy = results[spacing_mm]["state"][:, 4] * 1000  # mm/s
        analysis = results[spacing_mm]["analysis"]
        color = "red" if analysis["is_ping_ponging"] else colors[i % len(colors)]

        fig3.add_trace(
            go.Scatter(
                x=y,
                y=vy,
                mode="lines",
                name=f"{spacing_mm}mm",
                line=dict(color=color, width=2),
                hovertemplate=f"Spacing: {spacing_mm}mm<br>Position: %{{x:.2f}}mm<br>Velocity: %{{y:.2f}}mm/s<extra></extra>",
            )
        )

    fig3.update_layout(
        title="Phase Plot: Lateral Position vs Velocity",
        xaxis_title="Lateral Position (mm)",
        yaxis_title="Lateral Velocity (mm/s)",
        hovermode="closest",
        height=400,
        template="plotly_white",
    )
    
    # 4. Summary bar chart
    fig4 = go.Figure()
    spacing_labels = [f"{s}mm" for s in spacings]
    max_deviations = [results[s]["analysis"]["lateral_max"] * 1000 for s in spacings]
    colors_bar = [
        "red" if results[s]["analysis"]["is_ping_ponging"] else "green" for s in spacings
    ]

    fig4.add_trace(
        go.Bar(
            x=spacing_labels,
            y=max_deviations,
            marker_color=colors_bar,
            text=[f"{d:.2f}mm" for d in max_deviations],
            textposition="outside",
            hovertemplate="Spacing: %{x}<br>Max Deviation: %{y:.2f}mm<extra></extra>",
        )
    )

    fig4.update_layout(
        title="Maximum Lateral Deviation by Spacing",
        xaxis_title="Spacing (mm)",
        yaxis_title="Max Lateral Deviation (mm)",
        height=400,
        template="plotly_white",
    )

    # 5. Oscillation frequency chart
    fig5 = go.Figure()
    osc_freqs = [results[s]["analysis"]["oscillation_frequency"] for s in spacings]

    fig5.add_trace(
        go.Bar(
            x=spacing_labels,
            y=osc_freqs,
            marker_color=colors_bar,
            text=[f"{f:.2f} Hz" for f in osc_freqs],
            textposition="outside",
            hovertemplate="Spacing: %{x}<br>Frequency: %{y:.2f} Hz<extra></extra>",
        )
    )

    fig5.update_layout(
        title="Oscillation Frequency by Spacing",
        xaxis_title="Spacing (mm)",
        yaxis_title="Frequency (Hz)",
        height=400,
        template="plotly_white",
    )

    # 6. Maximum contact force chart
    fig6 = go.Figure()
    max_forces = [results[s]["analysis"]["max_contact_force"] / 1000 for s in spacings]  # Convert to kN
    colors_force = [
        "red" if results[s]["analysis"]["excessive_force"] else "green" for s in spacings
    ]

    fig6.add_trace(
        go.Bar(
            x=spacing_labels,
            y=max_forces,
            marker_color=colors_force,
            text=[f"{f:.1f} kN" for f in max_forces],
            textposition="outside",
            hovertemplate="Spacing: %{x}<br>Max Force: %{y:.1f} kN<extra></extra>",
        )
    )

    fig6.update_layout(
        title="Maximum Contact Force by Spacing",
        xaxis_title="Spacing (mm)",
        yaxis_title="Max Force (kN)",
        height=400,
        template="plotly_white",
    )

    # 7. Energy imparted chart
    fig7 = go.Figure()
    energies = [results[s]["analysis"]["energy_imparted"] for s in spacings]
    colors_energy = [
        "red" if results[s]["analysis"]["high_energy"] else "green" for s in spacings
    ]

    fig7.add_trace(
        go.Bar(
            x=spacing_labels,
            y=energies,
            marker_color=colors_energy,
            text=[f"{e:.1f} J" for e in energies],
            textposition="outside",
            hovertemplate="Spacing: %{x}<br>Energy: %{y:.1f} J<extra></extra>",
        )
    )

    fig7.update_layout(
        title="Energy Imparted to Rails by Spacing",
        xaxis_title="Spacing (mm)",
        yaxis_title="Energy (J)",
        height=400,
        template="plotly_white",
    )

    # 8. Climbing risk chart
    fig8 = go.Figure()
    climb_risks = [results[s]["analysis"]["climbing_risk"] * 100 for s in spacings]  # Convert to %
    colors_climb = [
        "red" if results[s]["analysis"]["climbing_risk_high"] else "green" for s in spacings
    ]

    fig8.add_trace(
        go.Bar(
            x=spacing_labels,
            y=climb_risks,
            marker_color=colors_climb,
            text=[f"{r:.0f}%" for r in climb_risks],
            textposition="outside",
            hovertemplate="Spacing: %{x}<br>Climbing Risk: %{y:.0f}%<extra></extra>",
        )
    )

    fig8.update_layout(
        title="Climbing Risk by Spacing",
        xaxis_title="Spacing (mm)",
        yaxis_title="Climbing Risk (%)",
        height=400,
        template="plotly_white",
    )
    
    # Create summary table HTML
    table_rows = [
        html.Tr([
            html.Th("Spacing (mm)"),
            html.Th("Ping-ponging"),
            html.Th("Max Lateral Dev (mm)"),
            html.Th("Max Force (kN)"),
            html.Th("Energy (J)"),
            html.Th("Climb Risk"),
            html.Th("Issues"),
        ])
    ]

    for row in summary_data:
        ping_color = "red" if row["Ping-ponging"] == "Yes" else "green"
        # Color code issues
        issues_text = row["Issues"] if row["Issues"] else "None"
        issues_color = "red" if row["Issues"] else "green"
        
        table_rows.append(
            html.Tr([
                html.Td(row["Spacing (mm)"]),
                html.Td(
                    row["Ping-ponging"],
                    style={"color": ping_color, "fontWeight": "bold"},
                ),
                html.Td(row["Max Lateral Dev (mm)"]),
                html.Td(row["Max Force (kN)"]),
                html.Td(row["Energy (J)"]),
                html.Td(row["Climb Risk"]),
                html.Td(
                    issues_text,
                    style={"color": issues_color, "fontWeight": "bold" if row["Issues"] else "normal"},
                ),
            ])
        )

    return html.Div([
        html.H2("Simulation Results", style={"marginTop": "30px", "marginBottom": "20px"}),
        html.Div([
            html.H3("Summary Table", style={"marginBottom": "15px"}),
            html.Table(
                table_rows,
                style={
                    "width": "100%",
                    "borderCollapse": "collapse",
                    "marginBottom": "30px",
                    "fontSize": "14px",
                },
            ),
        ], style={"marginBottom": "30px"}),
        html.Div([
            html.Div([dcc.Graph(figure=fig1)], style={"marginBottom": "30px"}),
            html.Div([dcc.Graph(figure=fig2)], style={"marginBottom": "30px"}),
            html.Div([dcc.Graph(figure=fig3)], style={"marginBottom": "30px"}),
            html.Div([
                html.Div(
                    [dcc.Graph(figure=fig4)],
                    style={"width": "48%", "display": "inline-block", "marginRight": "2%"},
                ),
                html.Div(
                    [dcc.Graph(figure=fig5)],
                    style={"width": "48%", "display": "inline-block"},
                ),
            ], style={"marginBottom": "30px"}),
            html.Div([
                html.Div(
                    [dcc.Graph(figure=fig6)],
                    style={"width": "48%", "display": "inline-block", "marginRight": "2%"},
                ),
                html.Div(
                    [dcc.Graph(figure=fig7)],
                    style={"width": "48%", "display": "inline-block"},
                ),
            ], style={"marginBottom": "30px"}),
            html.Div([
                html.Div(
                    [dcc.Graph(figure=fig8)],
                    style={"width": "48%", "display": "inline-block"},
                ),
            ], style={"marginBottom": "30px"}),
        ]),
    ])


if __name__ == "__main__":
    app.run_server(debug=True, port=8050)

