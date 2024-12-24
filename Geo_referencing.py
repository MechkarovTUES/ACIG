import numpy as np
import open3d as o3d
from processes.utils import get_geo_coordinates
# import open3d.visualization.gui as gui  
data = np.loadtxt("generated-cloud.txt")
points = data[:, :3]  # (x, y, z)
colors = data[:, 3:]  # (r, g, b)

# Filter NaN points
valid_mask = np.all(np.isfinite(points), axis=1)
points = points[valid_mask]
colors = colors[valid_mask]

# Convert any NaN in colors to 255 and then normalize from [0, 255] to [0, 1]
colors = np.nan_to_num(colors, nan=255) / 255.0

geo_coords = get_geo_coordinates(id=0)
print(geo_coords)
# Remove outliers in z using a threshold
scale_factor = 1e7
points[:, 0] = (points[:, 0] - geo_coords['longitude']) * scale_factor
points[:, 1] = (points[:, 1] - geo_coords["latitude"]) * scale_factor
points[:, 2] = points[:, 2] - points[:, 2].min()
z_threshold = 1
z = points[:, 2]
non_outlier_mask = np.abs(z - np.mean(z)) <= z_threshold * np.std(z)
points = points[non_outlier_mask]
colors = colors[non_outlier_mask]

# Normalize z values to start from 0
# points[:, 2] = points[:, 2] - points[:, 2].min()

# Create an Open3D PointCloud object
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
pcd.colors = o3d.utility.Vector3dVector(colors)
# pcd = pcd.voxel_down_sample(voxel_size=0.05) 



# pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
#     radius=1.0,  # adjust based on your point spacing
#     max_nn=30    # neighbors to consider
# ))

# # Optionally orient them so that the normals all generally point outward
# pcd.orient_normals_consistent_tangent_plane(100)
# The 100 is the number of neighbors used in this method.

# with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
#     mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
#         pcd,
#         depth=8,           # Increase for higher resolution
#         width=0, 
#         scale=1.1, 
#         linear_fit=False
#     )

# # Optional: Crop away low-density areas (outliers) if needed
# vertices_to_remove = densities < np.quantile(densities, 0.01)
# mesh.remove_vertices_by_mask(vertices_to_remove)

def pick_points_with_open3d(pcd):
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # Wait for user interaction
    vis.destroy_window()
    return vis.get_picked_points()

# Pick points interactively
picked_indices = pick_points_with_open3d(pcd)
print("Picked Points:")
for idx in picked_indices:
    x, y, z = points[idx]
    x = (x /scale_factor) + geo_coords['longitude']
    y = (y /scale_factor) + geo_coords['latitude']
    print(f"Index: {idx}, Coordinates: {x, y, z}")

# Visualize using Open3D's interactive viewer
# o3d.visualization.draw_geometries([mesh],
#                                   window_name="Open3D Point Cloud",
#                                   width=1024, height=768,
#                                   point_show_normal=False)
