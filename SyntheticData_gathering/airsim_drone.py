import cv2
import numpy as np
import json
import os
import cosysairsim as airsim

from time import sleep

def rgb2bgr(color):
    return [color[2], color[1], color[0]]

def begin_run(output_dir):
    if not os.path.exists(f"{output_dir}"):
        os.makedirs(f"{output_dir}")

    return f"{output_dir}"

def capture(dir, id):
    responses = client.simGetImages([cam_left])
    response = responses[0]
    img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)
    img_rgb = img1d.reshape(response.height, response.width, 3)
    img_rgb = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2RGB)
    cv2.imwrite(f"{dir}/im{id}_0.png", img_rgb)

    responses = client.simGetImages([cam_right])
    response = responses[0]
    img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)
    img_rgb = img1d.reshape(response.height, response.width, 3)
    img_rgb = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2RGB)
    cv2.imwrite(f"{dir}/im{id}_1.png", img_rgb)

    environment_state = client.simGetGroundTruthEnvironment()
    geo_coords = {
        "longitude": environment_state.geo_point.longitude,
        "latitude": environment_state.geo_point.latitude,
        "altitude": environment_state.geo_point.altitude,
        "height": -environment_state.position.z_val  # Height above ground
    }
    json_save(f"{dir}/im{id}_0.png", f"{dir}/im{id}_1.png", geo_coords)

def json_save(cam_left_path, cam_right_path, geo_coords):
    image_data = {
        "left": cam_left_path,
        "right": cam_right_path,
        "geo_coordinates": {
            "longitude": geo_coords['longitude'],
            "latitude": geo_coords['latitude'],
            "altitude": geo_coords['altitude'],
            "height": 10  # Include height if available
        }
    }
    data.append(image_data)

    with open(json_path, 'w') as file:
        json.dump(data, file, indent=4)

data = []
output_dir = "data"
json_path = "data.json"
global_id = 1
num_images_per_object = 30

if __name__ == "__main__":
    dir = begin_run(output_dir)
    cam_left = airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
    cam_right = airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)

    # segm_img_type = airsim.ImageRequest("0", airsim.ImageType.Segmentation, False, False)
    # ann_img_type = airsim.ImageRequest("0", airsim.ImageType.Annotation, False, False, annotation_name="VisTest")

    ip = "192.168.10.148"
    client = airsim.MultirotorClient(ip)
    client.confirmConnection()
    client.cancelLastTask()
    client.reset()
    client.enableApiControl(True)
    client.armDisarm(False)
    client.simGetImages([cam_left])
    client.simGetImages([cam_right])
    waypoints = [0, 5, -10]
    sleep(5)
    client.armDisarm(True)
    client.takeoffAsync().join()
    sleep(5)
    client.moveToZAsync(-10, 1).join()
    sleep(5)
    capture(dir, 1)
    sleep(5)
    client.moveToPositionAsync(waypoints[0], waypoints[1], waypoints[2], 1).join()
    sleep(5)
    capture(dir, 2)
    sleep(5)
    print(client.simGetGroundTruthEnvironment())
    #client.simSetCameraPose("0", airsim.Pose(airsim.Vector3r(100, 0, 0), airsim.Quaternionr()))
    