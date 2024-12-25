from .config import BASELINE, FOCAL_PX
from .utils import geo_coordinates_map
import numpy as np
import cv2


def get_depth_map(disparity_map):
    depth = np.zeros((*disparity_map.shape, 3))

    for row in range(disparity_map.shape[0]):
        for col in range(disparity_map.shape[1]):
            disparity_Px = disparity_map[row, col]
            # Calculate world coordinates using pre-derived formulas (similar triangles)
            if disparity_Px > 0:
                Z = ((BASELINE * FOCAL_PX) / (disparity_Px)) * -1
            else:
                Z = 0
            Y, X = geo_coordinates_map(cam = "right", x = col, y = row)
            # Y, X = row, col
            depth[row, col] = [X, Y, Z]
        progress = int((row / disparity_map.shape[0]) * 100)
        print(f"\rGenerating Depth cloud: {progress}% done", end="", flush=True)       
    return depth
        
def show_depth_map(depth_map):
    cv2.imshow("Depth Map", depth_map)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

