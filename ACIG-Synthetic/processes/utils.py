from PIL import Image, ImageFilter
from .config import SCALE, HEIGHT, ID, JSON_PATH, METERS_PER_DEGREE_LAT, H_FOV, \
V_FOV, TXT_PATH, PLY_PATH, DATASET
import numpy as np
import open3d as o3d
import os, json, math
from tqdm import tqdm

# ------------------IMAGE PROCESSING----------------

def get_image_arr(cam = "left", greyscale=False):
    """
    Loads an image from the dataset based on the camera type and returns it as a numpy array.
    - Parameters:
        cam: Camera type ('left' or 'right')
        greyscale: If True, converts the image to greyscale
    - Returns:
        img: Numpy array of the image
    """
    data = load_json(JSON_PATH)

    img = Image.open(f"{data[ID][cam]}")
    width, height = img.size
    img = img.resize((int(width/SCALE), int(height/SCALE)))
    if greyscale:
        img = img.convert(mode="L")
    else:
        if img.mode != "RGB":
            img = img.convert(mode="RGB")
    return np.asarray(img)

def load_json(json_file_path):
    """ Loads a JSON file and extracts relevant data.
    - Returns"
        result: List of dictionaries containing 'left', 'right', and 'geo_coordinates'
            *geo_coordinates: Dictionary with 'latitude' and 'longitude'
    """
    if not os.path.exists(json_file_path):
        raise FileNotFoundError(f"JSON_PATH file not found: {json_file_path}")

    with open(json_file_path, 'r') as file:
        data = json.load(file)

    result = []

    for entry in data:
        left = entry.get('left')
        right = entry.get('right')
        geo_coords = entry.get('geo_coordinates')
        result.append({"left": left, "right": right, "geo_coordinates": geo_coords})

    return result

def image_dimensions():
    """
    Returns the dimensions of the image in meters based on the camera parameters.
    - Returns:
        width_m: Width (Horizontal) of the image in meters
        height_m: Height (Vertical) of the image in meters
    """
    H = HEIGHT  
    half_w = H * math.tan(math.radians(H_FOV  / 2))
    half_h = H * math.tan(math.radians(V_FOV  / 2))
    return 2 * half_w, 2 * half_h

def get_geo_coordinates(id = 0):
    """Returns the geo-coordinates of the image with the given ID."""
    data = load_json(JSON_PATH)
    return data[id]['geo_coordinates']

def geo_coordinates_map(x, y, idx=ID):
    """
    Transforms pixel coordinates (x,y) → (latitude, longitude)
    using the geo-coordinates from the dataset.
    - Parameters:
        x: Pixel x-coordinate
        y: Pixel y-coordinate
        idx: Index of the image in the dataset (default is 0)
    """
    data = load_json(JSON_PATH)[idx]['geo_coordinates']
    img = Image.open(load_json(JSON_PATH)[idx]['right'])
    px_w, px_h = (s/SCALE for s in img.size)

    width_m, height_m = image_dimensions()
    mpp_x = width_m  / px_w
    mpp_y = height_m / px_h

    cx, cy = px_w/2, px_h/2
    dx, dy = x - cx, y - cy

    lat_per_px = mpp_y / METERS_PER_DEGREE_LAT
    lon_per_px = mpp_x / (
        METERS_PER_DEGREE_LAT * math.cos(math.radians(data['latitude']))
    )

    lat = data['latitude'] - dy * lat_per_px
    lon = data['longitude'] + dx * lon_per_px
    return lat, lon

# ---------------VISUALIZATION----------------

def load_point_clouds(voxel_size=3.0, num=1):
    '''
    Loads point clouds from text files and applies voxel downsampling.
    - Parameters:
        voxel_size: Size of the voxel for downsampling
        num: Number of point clouds to load
    '''
    pcds = []
    for i in range(num):
        pcd = load(f"{TXT_PATH}/{DATASET}/generated-cloud{i}.txt")
        pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
        pcds.append(pcd_down)
    return pcds

scale_factor=1e7
geo_coords = get_geo_coordinates(id=0)
def load(file_path, z_scaler=5):
    ''' Loads a point cloud from a .txt file and applies small post-processing.'''
    data = np.loadtxt(file_path)
    points = data[:, :3]  # (x, y, z)
    colors = data[:, 3:]  # (r, g, b)

    # Filter NaN values
    valid_mask = np.isfinite(points).all(axis=1)
    points = points[valid_mask]
    colors = colors[valid_mask]
    colors = np.nan_to_num(colors, nan=255) / 255.0

    points[:, 0] = (points[:, 0] - geo_coords['longitude']) * scale_factor
    points[:, 1] = (points[:, 1] - geo_coords["latitude"]) * scale_factor
    points[:, 2] = (points[:, 2] - points[:, 2].min()) * z_scaler

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    return pcd

def picked_points(picked_indices, pcd):
    print("Picked Points:")
    for idx in picked_indices:
        x, y, z = pcd.points[idx]
        x = (x /scale_factor) + geo_coords['longitude']
        y = (y /scale_factor) + geo_coords['latitude']
        #@TODO: Z axis should have a certain metric value, or alttiude or etc.
        print(f"Index: {idx}, Longitude: {x} Latitude: {y} | z: {z}")

def colored_icp_post_processing(source, target, transformation):
    ''' 
    Applies colored ICP post-processing to refine the alignment of two point clouds.
    - Parameters:
        source: Open3D PointCloud object (source point cloud)
        target: Open3D PointCloud object (target point cloud)
        transformation: Initial transformation matrix to align the source to the target
    '''
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
    #Colored ICP point cloud registration
    for scale in range(3):
        iter = max_iter[scale]
        radius = voxel_radius[scale]

        source_down = source.voxel_down_sample(radius)
        target_down = target.voxel_down_sample(radius)

        source_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
        target_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))

        result_icp = o3d.pipelines.registration.registration_colored_icp(
            source_down, target_down, 45, transformation,
            o3d.pipelines.registration.TransformationEstimationForColoredICP(),
            o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-10,
                                                            relative_rmse=1e-6,
                                                            max_iteration=iter))
        transformation = result_icp.transformation

    return transformation

def registration(pcds):
    ''' 
    Registers a list of point clouds using colored ICP post-processing.
    '''
    for id in range(len(pcds)):
        pcds[id].transform(np.identity(4))

    source = pcds[0]
    transformation = colored_icp_post_processing(pcds[0], pcds[1], np.identity(4))

    for num in tqdm(range(1, len(pcds))):
        transformation = colored_icp_post_processing(pcds[num-1], pcds[num], transformation)
        target = pcds[num]
        source.transform(transformation)
        source = source + target
    source.normals = o3d.utility.Vector3dVector()

    return source

def save_pointcloud(pcd, filename="output_pointcloud", export_format="ply"):
    ''' 
    Saves a point cloud to a file in the specified format.
    - Parameters:
        pcd: Open3D PointCloud object
        filename: Name of the output file (without extension)
        export_format: Format to save the point cloud ('ply' or 'xyz')
    '''
    if not os.path.exists("./PCDs"):
        os.makedirs("./PCDs")
    if not os.path.exists(PLY_PATH):
        os.makedirs(PLY_PATH)

    path = f"{PLY_PATH}/{filename}.{export_format}"
    if export_format in ["ply", "xyz"]:
        o3d.io.write_point_cloud(path, pcd)
        print(f"Point cloud saved as: {path}")
    else:
        raise ValueError("Invalid export format. Use 'ply' or 'xyz'.")

def crop_point_cloud(pcd, min_bound, max_bound):
    '''
    Crops a point cloud to the specified bounding box.
    - Parameters:
        pcd: Open3D PointCloud object
        min_bound: Minimum bound of the bounding box (3D point)
        max_bound: Maximum bound of the bounding box (3D point)
    '''
    bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=min_bound, max_bound=max_bound)
    return pcd.crop(bbox)

def create_poisson_mesh(pcd, depth=9):
    '''
    Creates a mesh from a point cloud using Poisson reconstruction.
    - Parameters:
        pcd: Open3D PointCloud object
        depth: Depth of the Poisson reconstruction (higher values yield more detail)
    '''
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

def visualize_point_cloud(pcd, name="Point Cloud Visualization"):
    '''
    Visualizes a point cloud using Open3D.
    '''
    o3d.visualization.draw_geometries([pcd], window_name=name)

