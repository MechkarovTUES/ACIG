import numpy as np
import cv2
from .config import FX_PX, BASELINE

def get_depth_map(disparity_map):
    depth = np.zeros((*disparity_map.shape, 3))

    for row in range(disparity_map.shape[0]):
        for col in range(disparity_map.shape[1]):
            disparity_Px = disparity_map[row, col]
            if disparity_Px > 0 and not np.isnan(disparity_Px):
                Z = disparity_Px
                Y, X = row, col
                # Z = FX_PX * BASELINE / Z if Z != 0 else 0
                depth[row, col] = [X, Y, Z]
            else:
                Z = 0
            
        progress = int((row / disparity_map.shape[0]) * 100)
        print(f"\rGenerating Depth cloud: {progress}% done", end="", flush=True)       
    return depth
        
def show_depth_map(depth_map):
    cv2.imshow("Depth Map", depth_map)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

