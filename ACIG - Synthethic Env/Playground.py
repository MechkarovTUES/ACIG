import open3d as o3d
import numpy as np
from processes.utils import get_geo_coordinates
from point_cloud_registration import ICP, PlaneICP, NDT, VPlaneICP
import copy

scale_factor = 1e7
geo_coords = get_geo_coordinates(id=0)

def load(file_path):
    data = np.loadtxt(file_path)
    points = data[:, :3]
    colors = data[:, 3:]

    valid_mask = np.isfinite(points).all(axis=1)
    points = points[valid_mask]
    colors = colors[valid_mask]
    colors = np.nan_to_num(colors, nan=255) / 255.0

    z_scaler = -1

    # points[:, 0] = (points[:, 0] - geo_coords['longitude']) * scale_factor
    # points[:, 1] = (points[:, 1] - geo_coords["latitude"]) * scale_factor
    points[:, 2] = (points[:, 2] - points[:, 2].min()) * z_scaler

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    return pcd

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    # source_temp.paint_uniform_color([1, 0.706, 0])
    # target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4559,
                                      front=[0.6452, -0.3036, -0.7011],
                                      lookat=[1.9892, 2.0208, 1.8945],
                                      up=[-0.2779, -0.9482, 0.1556])

def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def prepare_dataset(voxel_size):
    voxel_size = 3.0
    source = load("Presentation/generated-cloud0.txt")
    target = load("Presentation/generated-cloud1.txt")
    # trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
    trans_init = np.identity(4)
    source.transform(trans_init)
    # draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh

def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 15
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(True),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(10000000, 0.999999))
    return result

def refine_with_colored_icp(source, target, init_transformation, voxel_size):
    print("🔧 Refining with Colored ICP...")
    radius = voxel_size * 2

    source.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=30))
    target.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=30))

    result_icp = o3d.pipelines.registration.registration_colored_icp(
        source, target, voxel_size * 1.5, init_transformation,
        o3d.pipelines.registration.TransformationEstimationForColoredICP(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200)
    )
    return result_icp


# Usage Example
if __name__ == "__main__":
    voxel_size = 0.5  # means 5cm for this dataset
    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(
        voxel_size)

    icp = VPlaneICP(voxel_size=0.5, max_iter=30, max_dist=2, tol=1e-3)
    icp.set_target(target)
    T_new = icp.align(source, init_T=np.eye(4))
    draw_registration_result(source_down, target_down, T_new)
    # result_icp = refine_with_colored_icp(source, target, result_ransac.transformation, voxel_size)
    # draw_registration_result(source, target, result_icp.transformation)
