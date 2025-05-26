import numpy as np
import open3d as o3d
from processes.utils import get_geo_coordinates

# Global config
scale_factor = 1e7
geo_coords = get_geo_coordinates(id=0)

def load_point_cloud(file_path):
    # Load XYZRGB from .txt file
    data = np.loadtxt(file_path)
    points = data[:, :3]
    colors = data[:, 3:]

    # Remove NaNs
    valid_mask = np.isfinite(points).all(axis=1)
    points = points[valid_mask]
    colors = colors[valid_mask]

    # Normalize color range
    colors = np.nan_to_num(colors, nan=255) / 255.0

    # Coordinate transformation
    z_scaler = 3
    points[:, 0] = (points[:, 0] - geo_coords['longitude']) * scale_factor
    points[:, 1] = (points[:, 1] - geo_coords['latitude']) * scale_factor
    points[:, 2] = (points[:, 2] - points[:, 2].min()) * z_scaler

    # Create Open3D PointCloud object
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # Optional: Downsample
    pcd = pcd.voxel_down_sample(voxel_size=10.0)

    # Optional: Remove outliers
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

    return pcd

def save_pointcloud(pcd, filename="output_pointcloud", export_format="ply"):
    path = f"{filename}.{export_format}"
    if export_format in ["ply", "xyz"]:
        o3d.io.write_point_cloud(path, pcd)
        print(f"✅ Point cloud saved as: {path}")
    else:
        raise ValueError("Invalid export format. Use 'ply' or 'xyz'.")



if __name__ == "__main__":
    file_path = "Presentation/generated-cloud0.txt"  # 🔁 Replace with your actual file path

    # Load & process the cloud
    pcd = load_point_cloud(file_path)
    # Export to .ply for Blender (pure points, no mesh)
    save_pointcloud(pcd, export_format="ply")

    # Optional: visualize to verify
    o3d.visualization.draw_geometries([pcd])
