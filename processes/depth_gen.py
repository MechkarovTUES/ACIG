from .config import BASELINE, FOCAL_PX, SCALE
from .utils import geo_coordinates_map, geo_coordinates_map_2
import numpy as np
import cv2


def get_depth_map(disparity_map):
    depth = np.zeros((*disparity_map.shape, 3))

    for row in range(disparity_map.shape[0]):
        for col in range(disparity_map.shape[1]):
            disparity_Px = disparity_map[row, col]
            # Calculate world coordinates using pre-derived formulas (similar triangles). Z axis is inverted.
            if disparity_Px > 0:
                # Z = ((BASELINE * FOCAL_PX) / (disparity_Px)) * -1
                Z = disparity_Px
            else:
                Z = 0
                #@FIXME: Fix left/right swap.
            Y, X = geo_coordinates_map(x = col, y = row)
            # Y, X = row, col
            depth[row, col] = [X, Y, Z]
        progress = int((row / disparity_map.shape[0]) * 100)
        print(f"\rGenerating Depth cloud: {progress}% done", end="", flush=True)       
    return depth
        
def show_depth_map(depth_map):
    cv2.imshow("Depth Map", depth_map)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

