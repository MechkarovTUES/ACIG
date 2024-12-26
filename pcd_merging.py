import open3d as o3d
import numpy as np
from processes.utils import get_geo_coordinates
def load_pcd(file_path):
    data = np.loadtxt(file_path)
    points = data[:, :3]  # (x, y, z)
    colors = data[:, 3:]  # (r, g, b)

    # Filter NaN and z =0 points
    finite_mask = np.isfinite(points).all(axis=1)
    z_non_zero_mask = points[:, 2] != 0
    valid_mask = finite_mask & z_non_zero_mask
    points = points[valid_mask]
    colors = colors[valid_mask]

    z_threshold = 4
    z = points[:, 2]
    non_outlier_mask = np.abs(z - np.mean(z)) <= z_threshold * np.std(z)
    points = points[non_outlier_mask]
    colors = colors[non_outlier_mask]
    colors = np.nan_to_num(colors, nan=255) / 255.0

    geo_coords = get_geo_coordinates(id=0)
    print(geo_coords)

    scale_factor = 1e7
    if file_path == "generated-cloud2.txt":
        points[:, 0] = ((points[:, 0] - geo_coords['longitude']) * scale_factor) - 170
    else:
        points[:, 0] = (points[:, 0] - geo_coords['longitude']) * scale_factor
    points[:, 1] = (points[:, 1] - geo_coords["latitude"]) * scale_factor

    print("z before normalization:", points[:, 2].min(), points[:, 2].max())
    points[:, 2] = (points[:, 2] - points[:, 2].min()) * 3
    print("z after normalization:", points[:, 2].min(), points[:, 2].max())
    if file_path == "generated-cloud2.txt":
        points[:, 2] -= 100
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    print("Original point count:", len(pcd.points))
    down_pcd = pcd.voxel_down_sample(voxel_size=6.0)
    print("Downsampled point count:", len(down_pcd.points))

    return down_pcd

'''
def preprocess_point_cloud(pcd, voxel_size):
    pcd_down = pcd.voxel_down_sample(voxel_size)
    pcd_down.estimate_normals()
    radius_feature = voxel_size * 5
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100)
    )
    return pcd_down, pcd_fpfh

# RANdom SAmple Consensus (RANSAC) algorithm
def ransac_refine(pcd1_down, pcd2_down, pcd1_fpfh, pcd2_fpfh, distance_threshold):

    result_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        pcd1_down, pcd2_down,
        pcd1_fpfh, pcd2_fpfh,
        mutual_filter=True,
        max_correspondence_distance=distance_threshold,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        ransac_n=4,
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500)
    )
    return result_ransac

# Iterative Closest Point (ICP) algorithm
def icp_refine(pcd1, pcd2, distance_threshold, result_ransac):
    result_icp = o3d.pipelines.registration.registration_icp(
    pcd2, pcd1, distance_threshold, 
    result_ransac.transformation,
    o3d.pipelines.registration.TransformationEstimationPointToPlane()
)
    return result_icp
'''

pcd1 = load_pcd("generated-cloud1.txt")
pcd2 = load_pcd("generated-cloud2.txt")

o3d.visualization.draw_geometries([pcd1], window_name="PCD1")
o3d.visualization.draw_geometries([pcd2], window_name="PCD2")
merged_pcd = pcd1 + pcd2
# merged_pcd = merged_pcd.voxel_down_sample(voxel_size=3.0)
# o3d.visualization.draw_geometries([pcd2], window_name="After Transformation")
# o3d.visualization.draw_geometries([merged_pcd], window_name="Merged Point Cloud")
o3d.visualization.draw_geometries([pcd1, pcd2], window_name="Merged Point Cloud")
