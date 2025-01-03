import open3d as o3d
import numpy as np
import copy
from processes.utils import get_geo_coordinates

def load_point_clouds(voxel_size=0.0):
    pcds = []
    for i in range(2):
        pcd = load_pcd(f"generated-cloud{i}.txt", i)
        pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
        pcds.append(pcd_down)
    return pcds

def load_pcd(file_path, magic_scaler):
    data = np.loadtxt(file_path)
    points = data[:, :3]  # (x, y, z)
    colors = data[:, 3:]  # (r, g, b)

    # Filter NaN and z =0 points
    finite_mask = np.isfinite(points).all(axis=1)
    # z_non_zero_mask = points[:, 2] != 0
    valid_mask = finite_mask #& z_non_zero_mask
    points = points[valid_mask]
    colors = colors[valid_mask]

    # z_threshold = 4
    # z = points[:, 2]
    # non_outlier_mask = np.abs(z - np.mean(z)) <= z_threshold * np.std(z)
    # points = points[non_outlier_mask]
    # colors = colors[non_outlier_mask]
    colors = np.nan_to_num(colors, nan=255) / 255.0

    geo_coords = get_geo_coordinates(id=0)
    # print(geo_coords)
    # points = np.round(points, decimals=14)
    # print(points)
    scale_factor = 1e7
    # if file_path == "generated-cloud2.txt":
    #     points[:, 0] = ((points[:, 0] - geo_coords['longitude']) * scale_factor) - (debug_scaler*250)
    # elif file_path == "generated-cloud3.txt":
    #     points[:, 0] = ((points[:, 0] - geo_coords['longitude']) * scale_factor) - 500
    # else:
    points[:, 0] = ((np.round(points[:, 0], decimals=14) - geo_coords['longitude']) * scale_factor) #-(magic_scaler*270)
    points[:, 1] = (points[:, 1] - geo_coords["latitude"]) * scale_factor
    print("z before normalization:", points[:, 2].min(), points[:, 2].max())
    points[:, 2] = (points[:, 2] - points[:, 2].min()) * 4
    print("z after normalization:", points[:, 2].min(), points[:, 2].max())
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    print("Original point count:", len(pcd.points))
    down_pcd = pcd.voxel_down_sample(voxel_size=6.0)
    print("Downsampled point count:", len(down_pcd.points))

    return down_pcd

def draw_registration_result_original_color(source, target, transformation):
    source_temp = copy.deepcopy(source)
    source_temp.transform(transformation)
    target.normals = o3d.utility.Vector3dVector()
    source_temp.normals = o3d.utility.Vector3dVector()
    o3d.visualization.draw_geometries([source_temp, target],
                                      zoom=0.5,
                                      front=[-0.2458, -0.8088, 0.5342],
                                      lookat=[1.7745, 2.2305, 0.9787],
                                      up=[0.3109, -0.5878, -0.7468])
    
print("Load two point clouds and show initial pose")
pcds = load_point_clouds(voxel_size=6)
current_transformation = np.identity(4)
source = pcds[0]
target = pcds[1]



print("Estimate normals for source and target point clouds.")
source.estimate_normals(
    o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
target.estimate_normals(

    o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

print("2. Point-to-plane ICP registration is applied on original point")
print("   clouds to refine the alignment. Distance threshold 0.02.")
result_icp = o3d.pipelines.registration.registration_icp(
    source, target, 0.02, current_transformation,
    o3d.pipelines.registration.TransformationEstimationPointToPlane())
print(result_icp)
current_transformation = result_icp.transformation

voxel_radius = [10.0, 6.0, 3.00]
max_iter = [500, 300, 140]
print("Colored point cloud registration")
for scale in range(3):
    iter = max_iter[scale]
    radius = voxel_radius[scale]
    print([iter, radius, scale])

    print("4-1. Downsample with a voxel size %.2f" % radius)
    source_down = source.voxel_down_sample(radius)
    target_down = target.voxel_down_sample(radius)

    print("4-2. Estimate normal.")
    source_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
    target_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))

    print("4-3. Applying colored point cloud registration")
    result_icp = o3d.pipelines.registration.registration_colored_icp(
        source_down, target_down, radius, current_transformation,
        o3d.pipelines.registration.TransformationEstimationForColoredICP(),
        o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                          relative_rmse=1e-6,
                                                          max_iteration=iter))
    current_transformation = result_icp.transformation
    print(result_icp)
draw_registration_result_original_color(source, target,
                                        result_icp.transformation)