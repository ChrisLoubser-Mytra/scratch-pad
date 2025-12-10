"""
Web application for Mytra Robot Guide Wheel Spacing Analysis

Interactive dashboard to visualize and analyze simulation results.
"""

from typing import Any, Dict, List

import dash
from dash import dcc, html, Input, Output, State
from dash.exceptions import PreventUpdate
import numpy as np
import plotly.express as px
import plotly.graph_objs as go

from robot_simulation import RobotParams, RobotSimulator, run_spacing_analysis


# Initialize Dash app
app = dash.Dash(__name__)
app.title = "Mytra Robot Guide Wheel Spacing Analysis"

# Define app layout
app.layout = html.Div([
    html.Div([
        html.H1("Mytra Robot Guide Wheel Spacing Analysis", 
                style={'textAlign': 'center', 'marginBottom': '30px'}),
        
        html.Div([
            html.Div([
                html.Label("Spacing Values (mm, comma-separated):", 
                          style={'fontWeight': 'bold', 'marginBottom': '5px'}),
                dcc.Input(
                    id='spacing-input',
                    type='text',
                    value='5,10,20,30,40',
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
                html.Label("Initial Misalignment (rad):", 
                          style={'fontWeight': 'bold', 'marginBottom': '5px'}),
                dcc.Input(
                    id='theta-input',
                    type='number',
                    value=0.01,
                    min=0.0,
                    max=0.1,
                    step=0.001,
                    style={'width': '100%', 'padding': '8px'}
                ),
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
    [Output("results-container", "children"), Output("status-message", "children")],
    [Input("run-button", "n_clicks")],
    [State("spacing-input", "value"), State("duration-input", "value"), State("theta-input", "value")],
)
def update_results(
    n_clicks: int | None, spacing_str: str, duration: float, initial_theta: float
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

        if initial_theta < 0 or initial_theta > 0.1:
            return [], html.Div(
                "Error: Initial misalignment must be between 0 and 0.1 rad.",
                style={"color": "red"},
            )

        # Run simulation
        results = run_spacing_analysis(spacings, duration=duration, initial_theta=initial_theta)

        # Create visualizations
        status_msg = html.Div(
            f"Simulation complete! Analyzed {len(spacings)} spacing values.",
            style={"color": "green"},
        )

        return create_results_layout(results, spacings), status_msg

    except Exception as e:
        error_msg = f"Error: {str(e)}"
        return [], html.Div(error_msg, style={"color": "red"})


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
        })
    
    # 1. Lateral position over time (all spacings)
    fig1 = go.Figure()
    colors = px.colors.qualitative.Set1
    for i, spacing_mm in enumerate(spacings):
        t = results[spacing_mm]["time"]
        y = results[spacing_mm]["state"][:, 1] * 1000  # Convert to mm
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
        theta = results[spacing_mm]["state"][:, 2] * 180 / np.pi  # Convert to degrees
        analysis = results[spacing_mm]["analysis"]
        color = "red" if analysis["is_ping_ponging"] else colors[i % len(colors)]

        fig2.add_trace(
            go.Scatter(
                x=t,
                y=theta,
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
    
    # Create summary table HTML
    table_rows = [
        html.Tr([
            html.Th("Spacing (mm)"),
            html.Th("Ping-ponging"),
            html.Th("Max Lateral Dev (mm)"),
            html.Th("Lateral Std Dev (mm)"),
            html.Th("Oscillation Freq (Hz)"),
            html.Th("Growing Oscillations"),
        ])
    ]

    for row in summary_data:
        ping_color = "red" if row["Ping-ponging"] == "Yes" else "green"
        table_rows.append(
            html.Tr([
                html.Td(row["Spacing (mm)"]),
                html.Td(
                    row["Ping-ponging"],
                    style={"color": ping_color, "fontWeight": "bold"},
                ),
                html.Td(row["Max Lateral Dev (mm)"]),
                html.Td(row["Lateral Std Dev (mm)"]),
                html.Td(row["Oscillation Freq (Hz)"]),
                html.Td(row["Growing"]),
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
        ]),
    ])


if __name__ == "__main__":
    app.run_server(debug=True, port=8050)

