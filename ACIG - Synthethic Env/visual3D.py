import open3d as o3d
import numpy as np
from processes.utils import get_geo_coordinates
from point_cloud_registration import VPlaneICP

def load_point_clouds(folder="Generated_PCDs", voxel_size=3.0, num=1):
    pcds = []
    for i in range(num):
        pcd = load(f"{folder}/generated-cloud{i}.txt")
        pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
        pcds.append(pcd_down)
    return pcds

scale_factor = 1e7
geo_coords = get_geo_coordinates(id=0)

def load(file_path):
    data = np.loadtxt(file_path)
    points = data[:, :3]  # (x, y, z)
    colors = data[:, 3:]  # (r, g, b)

    # Filter NaN values
    valid_mask = np.isfinite(points).all(axis=1)
    points = points[valid_mask]
    colors = colors[valid_mask]
    colors = np.nan_to_num(colors, nan=255) / 255.0

    z_scaler = 2.5

    points[:, 0] = (points[:, 0] - geo_coords['longitude']) * scale_factor
    points[:, 1] = (points[:, 1] - geo_coords["latitude"]) * scale_factor
    points[:, 2] = (points[:, 2] - points[:, 2].min()) * z_scaler

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    return pcd

def create_mesh_from_point_cloud(pcd):
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=30))

    distances = pcd.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    radius = 2 * avg_dist 

    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        pcd,
        o3d.utility.DoubleVector([radius, radius * 2])
    )
    return mesh

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

def export_mesh_to_blender(mesh, file_path="ACIG_exmpl_mesh.ply"):
    """
    Exports the given Open3D mesh to a file compatible with Blender.

    Parameters:
        mesh (o3d.geometry.TriangleMesh): The mesh to export.
        file_path (str): The path to save the mesh file (should end in .ply or .obj).
    """
    # Make sure the mesh has vertex colors or Blender will render it plain grey
    if not mesh.has_vertex_colors():
        # Optional: Assign uniform gray color if none present
        gray = np.ones((np.asarray(mesh.vertices).shape[0], 3)) * 0.7
        mesh.vertex_colors = o3d.utility.Vector3dVector(gray)

    # Export the mesh
    o3d.io.write_triangle_mesh(file_path, mesh)
    print(f"Mesh successfully exported to: {file_path}")

def crop_point_cloud(pcd, min_bound, max_bound):
    bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=min_bound, max_bound=max_bound)
    return pcd.crop(bbox)

def geo_reference(pcd):
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()
    vis.destroy_window()
    return vis.get_picked_points()

# Load point clouds
pcds = load_point_clouds(folder="Presentation", voxel_size=5, num=4)
source = pcds[3]
source = crop_point_cloud(pcd=source, min_bound=[-400, -1000, -1000], max_bound=[820, 1000, 1000])
# o3d.io.write_point_cloud("cloud_env2.ply", source)
# Create mesh from point cloud
# mesh = create_mesh_from_point_cloud(source)

# mesh = create_poisson_mesh(source, depth=12)
# Export mesh to Blender
# export_mesh_to_blender(mesh, file_path="ACIG_exmpl_mesh.ply")
# mesh.vertex_normals = o3d.utility.Vector3dVector() 

# o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)
# Visualize and geo-reference
picked_indices = geo_reference(source)
print("Picked Points:")
for idx in picked_indices:
    x, y, z = source.points[idx]
    lon = (x / scale_factor) + geo_coords['longitude']
    lat = (y / scale_factor) + geo_coords['latitude']
    print(f"Index: {idx}, Longitude: {lon} Latitude: {lat} | z: {z}")
