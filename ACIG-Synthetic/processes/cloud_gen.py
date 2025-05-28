import numpy as np
from .config import ID, DATASET, TXT_PATH
import os
def get_points(distances, image_arr):
    points = []
    # Maps every pixel in the depth map to the corresponding color information of the image
    for row in range(distances.shape[0]): 
        for col in range(distances.shape[1]):
            [r, g, b] = image_arr[row, col]
            [X, Y, Z] = distances[row, col]

            if Z is None:
                continue
            elif r > 230 and g < 10 and b < 10:
                for i in range(0,11):
                    points.append([X, Y, Z+i, r, g, b])
            else: 
                points.append([X, Y, Z, r, g, b]) 
        progress = int((row / distances.shape[0]) * 100) 
        print(f"\rGenerating Point cloud: {progress}% done", end="", flush=True)

    # Check for missing directories and create them if necessary   
    if not os.path.exists("./PCDs"):
        os.makedirs("./PCDs")
    if not os.path.exists(f"./{TXT_PATH}"):
        os.makedirs(f"./{TXT_PATH}")
    if not os.path.exists(f"./{TXT_PATH}/{DATASET}"):
        os.makedirs(f"./{TXT_PATH}/{DATASET}")

    np.savetxt(f"./{TXT_PATH}/{DATASET}/generated-cloud{ID}.txt", points) 
    return points  
