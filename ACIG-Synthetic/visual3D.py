import open3d as o3d
import numpy as np
from processes.utils import load_point_clouds, crop_point_cloud
from point_cloud_registration import VPlaneICP


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
