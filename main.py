from processes.utils import get_image_arr
from processes.depth_gen import get_depth_map
from processes.cloud_gen import get_points
from processes.disparity_gen import get_disparity_map, show_disparity_map
import numpy as np


def main(file_out = "./generated-cloud.txt"):
    left_image_arr = get_image_arr(id=0, cam="right", greyscale=True)
    right_image_arr = get_image_arr(id=0, cam="left", greyscale=True)
    left_image_arr_color = get_image_arr(id=0, cam="right", greyscale=False)

    disparity_map = get_disparity_map(left_image_arr, right_image_arr, 8)
    show_disparity_map(disparity_map)
    distance_map = get_depth_map(disparity_map)
    points = get_points(distance_map, left_image_arr_color)
    np.savetxt(file_out, points)    
    
if __name__ == "__main__": 
    main()