import cv2
import numpy as np
import json
import os
import cosysairsim as airsim
from time import sleep

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
    cv2.imwrite(f"{dir}/im{id}_1.png", img_rgb)

    responses = client.simGetImages([cam_right])
    response = responses[0]
    img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)
    img_rgb = img1d.reshape(response.height, response.width, 3)
    img_rgb = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2RGB)
    cv2.imwrite(f"{dir}/im{id}_0.png", img_rgb)

    environment_state = client.simGetGroundTruthEnvironment()
    geo_coords = {
        "longitude": environment_state.geo_point.longitude,
        "latitude": environment_state.geo_point.latitude,
        "altitude": environment_state.geo_point.altitude,
        "height": -environment_state.position.z_val  # Height above ground
    }
    json_save(f"{dir}/im{id}_1.png", f"{dir}/im{id}_0.png", geo_coords)

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
output_dir = "test"
json_path = "test.json"
global_id = 1
num_images_per_object = 30

if __name__ == "__main__":
    dir = begin_run(output_dir)
    cam_left = airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
    cam_right = airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)

    ip = ""

    client = airsim.MultirotorClient(ip)
    client.confirmConnection()
    client.cancelLastTask()
    client.reset()
    client.enableApiControl(True)
    client.armDisarm(False)

    client.simGetImages([cam_left])
    client.simGetImages([cam_right])

    waypoints1 = [[24, -24, -10],[24, -16, -10],[24, -8, -10],[24, 0, -10],[24, 8, -10],[24, 16, -10],[24, 24, -10],
                  [16, -24, -10],[16, -16, -10],[16, -8, -10],[16, 0, -10],[16, 8, -10],[16, 16, -10],[16, 24, -10],
                  [8, -24, -10],[8, -16, -10],[8, -8, -10],[8, 0, -10],[8, 8, -10],[8, 16, -10],[8, 24, -10],
                  [0, -24, -10],[0, -16, -10],[0, -8, -10],[0, 0, -10],[0, 8, -10],[0, 16, -10],[0, 24, -10],
                  [-8, -24, -10],[-8, -16, -10],[-8, -8, -10],[-8, 0, -10],[-8, 8, -10],[-8, 16, -10],[-8, 24, -10],
                  [-16, -24, -10],[-16, -16, -10],[-16, -8, -10],[-16, 0, -10],[-16, 8, -10],[-16, 16, -10],[-16, 24, -10],
                  [-24, -24, -10],[-24, -16, -10],[-24, -8, -10],[-24, 0, -10],[-24, 8, -10],[-24, 16, -10],[-24, 24, -10],]
    sleep(5)
    
    client.armDisarm(True)
    client.takeoffAsync().join()
    sleep(5)

    client.moveToZAsync(-10, 1).join()
    sleep(5)

    for i in range(len(waypoints1)):
        client.moveToPositionAsync(waypoints1[i][0], waypoints1[i][1], waypoints1[i][2], 1).join()
        sleep(5)
        capture(dir, i+1)
        sleep(5)
        print(client.simGetGroundTruthEnvironment())

    