import open3d as o3d
import numpy as np
from processes.utils import load_point_clouds, visualize_point_cloud, save_pointcloud

pcds = load_point_clouds(voxel_size=3.0, num=1)
source = pcds[0]

visualize_point_cloud(source, "Source Point Cloud")

# Save the point cloud to a file
save_pointcloud(source, filename="ACIG_exampl2", export_format="ply")