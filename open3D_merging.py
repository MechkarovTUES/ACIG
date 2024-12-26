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
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    print("Original point count:", len(pcd.points))
    down_pcd = pcd.voxel_down_sample(voxel_size=6.0)
    print("Downsampled point count:", len(down_pcd.points))

    return down_pcd

def load_point_clouds(voxel_size=0.0):
    pcds = []
    for i in range(2):
        pcd = load_pcd(f"generated-cloud{i+1}.txt")
        pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
        pcds.append(pcd_down)
    return pcds

def pairwise_registration(source, target):
    print("Apply point-to-plane ICP")
    icp_coarse = o3d.pipelines.registration.registration_icp(
        source, target, max_correspondence_distance_coarse, np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    icp_fine = o3d.pipelines.registration.registration_icp(
        source, target, max_correspondence_distance_fine,
        icp_coarse.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    transformation_icp = icp_fine.transformation
    information_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
        source, target, max_correspondence_distance_fine,
        icp_fine.transformation)
    return transformation_icp, information_icp


def full_registration(pcds, max_correspondence_distance_coarse,
                      max_correspondence_distance_fine):
    pose_graph = o3d.pipelines.registration.PoseGraph()
    odometry = np.identity(4)
    pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))
    n_pcds = len(pcds)
    for source_id in range(n_pcds):
        for target_id in range(source_id + 1, n_pcds):
            transformation_icp, information_icp = pairwise_registration(
                pcds[source_id], pcds[target_id])
            print("Build o3d.pipelines.registration.PoseGraph")
            if target_id == source_id + 1:  # odometry case
                odometry = np.dot(transformation_icp, odometry)
                pose_graph.nodes.append(
                    o3d.pipelines.registration.PoseGraphNode(
                        np.linalg.inv(odometry)))
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                             target_id,
                                                             transformation_icp,
                                                             information_icp,
                                                             uncertain=False))
            else:  # loop closure case
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                             target_id,
                                                             transformation_icp,
                                                             information_icp,
                                                             uncertain=True))
    return pose_graph

def pick_points_with_open3d(pcd):
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # Wait for user interaction
    vis.destroy_window()
    return vis.get_picked_points()

# Pick points interactively

scale_factor = 1e7
voxel_size = 6.0
pcds_down = load_point_clouds(voxel_size)

for pcd in pcds_down:
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    # pcd.normalize_normals()

print("Full registration ...")
max_correspondence_distance_coarse = voxel_size * 15
max_correspondence_distance_fine = voxel_size * 1.5
with o3d.utility.VerbosityContextManager(
        o3d.utility.VerbosityLevel.Debug) as cm:
    pose_graph = full_registration(pcds_down,
                                   max_correspondence_distance_coarse,
                                   max_correspondence_distance_fine)
print("Optimizing PoseGraph ...")
option = o3d.pipelines.registration.GlobalOptimizationOption(
    max_correspondence_distance=max_correspondence_distance_fine,
    edge_prune_threshold=0.25,
    reference_node=0)
with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
    o3d.pipelines.registration.global_optimization(
        pose_graph,
        o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
        o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
        option)

print("Transform points and display")
pcd_combined = o3d.geometry.PointCloud()
for point_id in range(len(pcds_down)):
    print(pose_graph.nodes[point_id].pose)
    pcds_down[point_id].transform(pose_graph.nodes[point_id].pose)
    pcds_down[point_id].normals = o3d.utility.Vector3dVector()
    pcd_combined += pcds_down[point_id]




pcd_combined_down = pcd_combined.voxel_down_sample(voxel_size=voxel_size)
# o3d.io.write_point_cloud("multiway_registration.pcd", pcd_combined_down)
# o3d.visualization.draw_geometries([pcd_combined_down],
#                                   zoom=0.3412,
#                                   front=[0.4257, -0.2125, -0.8795],
#                                   lookat=[2.6172, 2.0475, 1.532],
#                                   up=[-0.0694, -0.9768, 0.2024])
geo_coords = get_geo_coordinates(id=0)
picked_indices = pick_points_with_open3d(pcd_combined_down)
print("Picked Points:")
for idx in picked_indices:
    x, y, z = pcd_combined_down.points[idx]
    x = (x /scale_factor) + geo_coords['longitude']
    y = (y /scale_factor) + geo_coords['latitude']
    print(f"Index: {idx}, Longitude: {x} Latitude: {y} | z: {z}")

# o3d.visualization.draw_geometries(pcds_down,
#                                   zoom=0.3412,
#                                   front=[0.4257, -0.2125, -0.8795],
#                                   lookat=[2.6172, 2.0475, 1.532],
#                                   up=[-0.0694, -0.9768, 0.2024])