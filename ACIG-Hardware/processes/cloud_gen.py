import numpy as np
import os
from .config import ID
def get_points(distances, image_arr):
    points = []
    # Checks every pixel in the depth map
    for row in range(distances.shape[0]): 
        for col in range(distances.shape[1]):
            [r, g, b] = image_arr[row, col]
            [X, Y, Z] = distances[row, col]

            if Z is None:
                continue
            else: 
                points.append([X, Y, Z, r, g, b]) 
        progress = int((row / distances.shape[0]) * 100) 
        print(f"\rGenerating Point cloud: {progress}% done", end="", flush=True)
        
    if not os.path.exists("./PCDs"):
        os.makedirs("./PCDs")
    np.savetxt(f"./PCDs/generated-cloud{ID}.txt", points) 
    return points  



