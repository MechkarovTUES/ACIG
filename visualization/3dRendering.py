import matplotlib.pyplot as plt
import numpy as np

# Load the point cloud data
data = np.loadtxt("generated-cloud.txt")

# Extract points and colors
points = data[:, :3]  # First three columns are points (x, y, z)
colors = data[:, 3:]  # Next three columns are color values (r, g, b)

# Remove invalid points (NaN or Inf in any coordinate)
valid_mask = np.all(np.isfinite(points), axis=1)
points = points[valid_mask]
colors = colors[valid_mask]

# Extract x, y, z
x = points[:, 0]
y = points[:, 1]
z = points[:, 2]

# Normalize the colors to the range [0, 1]
colors = np.nan_to_num(colors, nan=255) / 255.0

# --- 2D Visualization of Point Cloud Density ---
plt.figure(figsize=(10, 6))
plt.scatter(x, y, s=1, c=z, cmap='viridis')
plt.colorbar(label="Z Values (Height)")
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Point Cloud (2D Density)')
plt.show()

# --- 3D Visualization of Point Cloud ---
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot the 3D point cloud
ax.scatter(x, y, z, s=1, c=colors, marker='o')

# Optionally, set axis labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.title('Point Cloud (3D Scatter Plot)')

plt.show()