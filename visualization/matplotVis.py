import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import griddata

def setup_plot():
    x_range = x.max() - x.min()
    y_range = y.max() - y.min()
    z_range = z.max() - z.min()

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_surface(grid_x, grid_y, grid_z, facecolors=grid_colors, edgecolor='none')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    ax.set_box_aspect([x_range, y_range, z_range])

data = np.loadtxt("generated-cloud.txt")
points = data[:, :3]  #(x, y, z)
colors = data[:, 3:]  #(r, g, b)

# Filter NaN points
valid_mask = np.all(np.isfinite(points), axis=1)
points = points[valid_mask]
colors = colors[valid_mask]


z = points[:, 2]
colors = np.nan_to_num(colors, nan=255) / 255.0

z_threshold = 1
non_outlier_mask = np.abs(z - np.mean(z)) <= z_threshold * np.std(z)
points = points[non_outlier_mask]
colors = colors[non_outlier_mask]

x = points[:, 0]
y = points[:, 1]
z = points[:, 2]

scale_factor = 1  # Change this factor to increase or decrease the elevation
z = z - z.min() * scale_factor

grid_resolution = 250
aspect_ratio = (x.max() - x.min()) / (y.max() - y.min())
n_points_y = grid_resolution
n_points_x = int(grid_resolution * aspect_ratio)

grid_x, grid_y = np.meshgrid(
    np.linspace(x.min(), x.max(), n_points_x),
    np.linspace(y.min(), y.max(), n_points_y) 
)

grid_z = griddata(points[:, :2], z, (grid_x, grid_y), method='linear')
grid_r = griddata(points[:, :2], colors[:, 0], (grid_x, grid_y), method='linear')
grid_g = griddata(points[:, :2], colors[:, 1], (grid_x, grid_y), method='linear')
grid_b = griddata(points[:, :2], colors[:, 2], (grid_x, grid_y), method='linear')
grid_colors = np.stack([grid_r, grid_g, grid_b], axis=-1)

setup_plot()
plt.show()