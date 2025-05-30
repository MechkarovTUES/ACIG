import open3d as o3d
import numpy as np


def load_point_clouds(voxel_size=3.0, num=1):
    pcds = []
    for i in range(num):
        pcd = load(f"PCDs/generated-cloud{i}.txt")
        pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
        pcds.append(pcd_down)
    return pcds

def load(file_path):
    data = np.loadtxt(file_path)
    points = data[:, :3]  # (x, y, z)
    colors = data[:, 3:]  # (r, g, b)
    # points[:, 2] = points[:, 2] > points[:, 2].min()  # Filter out points with z <=0

    # Filter NaN values
    valid_mask = np.isfinite(points).all(axis=1)
    points = points[valid_mask]
    colors = colors[valid_mask]
    colors = np.nan_to_num(colors, nan=255) / 255.0

    z_scaler = -15
    points[:, 2] = (points[:, 2] - points[:, 2].min()) * z_scaler

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
#     pcd, ind = pcd.remove_statistical_outlier(
#     nb_neighbors=300,
#     std_ratio=1.0
# )
    return pcd

def create_poisson_mesh(pcd, depth=9):
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=30))
    
    # Optional: Orients normals consistently (important for watertight mesh)
    pcd.orient_normals_consistent_tangent_plane(100)

    # Generate mesh
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=depth)

    # Remove low-density vertices (optional clean-up step)
    densities = np.asarray(densities)
    density_threshold = np.quantile(densities, 0.02)
    vertices_to_keep = densities > density_threshold
    mesh.remove_vertices_by_mask(~vertices_to_keep)

    mesh.compute_vertex_normals()
    return mesh

pcds = load_point_clouds(voxel_size=1.0, num=5)

source = pcds[0]
# source = create_poisson_mesh(source, depth=9) 
source.normals = o3d.utility.Vector3dVector()

def geo_reference(pcd):
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # Wait for user interaction
    vis.destroy_window()
    return vis.get_picked_points()


# picked_indices = geo_reference(source)
# print("Picked Points:")
# for idx in picked_indices:
#     x, y, z = source.points[idx]
#     print(f"Index: {idx}, X: {x} Y: {y} | Depth: {z}")

# source = create_poisson_mesh(source, depth=18)
o3d.visualization.draw_geometries([source], window_name="Point Cloud", width=800, height=600, left=50, top=50, mesh_show_back_face=True)