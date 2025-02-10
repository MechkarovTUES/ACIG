import open3d as o3d
import numpy as np
from processes.utils import get_geo_coordinates

def load_point_clouds(voxel_size=3.0, num=1):
    pcds = []
    for i in range(num):
        pcd = load(f"Generated_PCDs/generated-cloud{i}.txt")
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

    z_scaler = 5

    points[:, 0] = (points[:, 0] - geo_coords['longitude']) * scale_factor
    points[:, 1] = (points[:, 1] - geo_coords["latitude"]) * scale_factor
    points[:, 2] = (points[:, 2] - points[:, 2].min()) * z_scaler

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    return pcd

def colored_icp_post_processing(source, target, transformation):
    #Estimate normals for source and target point clouds
    source.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    target.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    
    #Point-to-plane ICP registration is applied on original point clouds to refine the alignment
    result_icp = o3d.pipelines.registration.registration_icp(
        source, target, 0.9, transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    transformation = result_icp.transformation

    voxel_radius = [5.0, 5.0, 5.0]
    max_iter = [50, 30, 14]
    #Colored point cloud registration
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
            source_down, target_down, 45, transformation,
            o3d.pipelines.registration.TransformationEstimationForColoredICP(),
            o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-10,
                                                            relative_rmse=1e-6,
                                                            max_iteration=iter))
        transformation = result_icp.transformation
        print(result_icp)
    return transformation

pcds = load_point_clouds(voxel_size=1.0, num=7)
current_transformation = np.identity(4)
for id in range(len(pcds)):
    pcds[id].transform(current_transformation)

source = pcds[0]
transformation = colored_icp_post_processing(pcds[0], pcds[1], np.identity(4))

for num in range(1, len(pcds)):
    transformation = colored_icp_post_processing(pcds[num-1], pcds[num], transformation)
    target = pcds[num]
    source.transform(transformation)
    source = source + target
source.normals = o3d.utility.Vector3dVector()

def geo_reference(pcd):
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # Wait for user interaction
    vis.destroy_window()
    return vis.get_picked_points()

picked_indices = geo_reference(source)
print("Picked Points:")
for idx in picked_indices:
    x, y, z = source.points[idx]
    x = (x /scale_factor) + geo_coords['longitude']
    y = (y /scale_factor) + geo_coords['latitude']
    #@TODO: Z axis should have a certain metric value, or alttiude or etc.
    print(f"Index: {idx}, Longitude: {x} Latitude: {y} | z: {z}")


