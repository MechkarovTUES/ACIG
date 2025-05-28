import numpy as np
import open3d as o3d
from processes.utils import get_geo_coordinates


scale_factor = 1e7
geo_coords = get_geo_coordinates(id=0)


def load_point_cloud(file_path):
    data = np.loadtxt(file_path)
    points = data[:, :3]
    colors = data[:, 3:]

    # Remove NaN values
    valid_mask = np.isfinite(points).all(axis=1)
    points = points[valid_mask]
    colors = colors[valid_mask]
    colors = np.nan_to_num(colors, nan=255) / 255.0

    # Coordinate conversion
    z_scaler = 3
    points[:, 0] = (points[:, 0] - geo_coords['longitude']) * scale_factor
    points[:, 1] = (points[:, 1] - geo_coords['latitude']) * scale_factor
    points[:, 2] = (points[:, 2] - points[:, 2].min()) * z_scaler

    # Create Open3D PointCloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # Downsample
    pcd = pcd.voxel_down_sample(voxel_size=10.0)

    # Remove outliers
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

    return pcd


def pointcloud_to_mesh(pcd, method="poisson"):
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=30))

    if method == "poisson":
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=7)
        # Optional: Filter mesh by density (clean fuzzy parts)
        densities = np.asarray(densities)
        mask = densities > np.quantile(densities, 0.02)
        mesh.remove_vertices_by_mask(~mask)
    elif method == "ball_pivoting":
        radii = [0.5, 1.0, 1.5, 2.0]
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
            pcd, o3d.utility.DoubleVector(radii)
        )
    else:
        raise ValueError("Invalid meshing method. Use 'poisson' or 'ball_pivoting'.")

    mesh.compute_vertex_normals()
    return mesh


def crop_mesh_to_pointcloud(mesh, pcd):
    bbox = pcd.get_axis_aligned_bounding_box()
    return mesh.crop(bbox)


def save_mesh(mesh, filename="output_mesh", export_format="ply"):
    path = f"{filename}.{export_format}"
    if export_format == "ply":
        o3d.io.write_triangle_mesh(path, mesh)
    elif export_format == "stl":
        o3d.io.write_triangle_mesh(path, mesh)
    elif export_format == "glb":
        o3d.io.write_triangle_mesh(path, mesh)
    else:
        raise ValueError("Invalid export format. Use 'ply', 'stl', or 'glb'.")
    print(f"✅ Mesh saved as: {path}")


if __name__ == "__main__":
    file_path = "Presentation/generated-cloud0.txt"  # change if needed
    pcd = load_point_cloud(file_path)
    mesh = pointcloud_to_mesh(pcd, method="poisson")
    mesh = crop_mesh_to_pointcloud(mesh, pcd)
    save_mesh(mesh, export_format="ply")
    o3d.visualization.draw_geometries([mesh])
