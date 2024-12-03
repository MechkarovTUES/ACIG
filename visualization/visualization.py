import numpy as np
import plotly.graph_objects as go

# Load the point cloud data from the txt file
data = np.loadtxt("generated-cloud.txt")

# Extract points and colors
points = data[:, :3]  # First three columns are points (x, y, z)
colors = data[:, 3:]  # Next three columns are color values (r, g, b)

# Normalize the colors to the range [0, 1]
colors = np.nan_to_num(colors, nan=255) / 255.0

# Create the point cloud plot
fig = go.Figure(data=[go.Scatter3d(
    x=points[:, 0], 
    y=points[:, 1], 
    z=points[:, 2],
    mode='markers',
    marker=dict(
        size=3,
        color=colors,  # Set colors based on your data
        opacity=0.8
    )
)])

# Set the layout for the plot
fig.update_layout(
    title="3D Point Cloud Visualization",
    scene=dict(
        xaxis_title='X',
        yaxis_title='Y',
        zaxis_title='Z'
    ),
    margin=dict(l=0, r=0, b=0, t=0),
)

# Show the plot in the browser
fig.show()