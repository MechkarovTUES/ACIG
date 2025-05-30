from processes.utils import get_image_arr, rectify_set, grayscale, resize, show_images, crop
from processes.depth_gen import get_depth_map
from processes.cloud_gen import get_points
from processes.disparity_gen import get_disparity_map, show_disparity_map
import numpy as np

def main():
    left = get_image_arr(cam="left")
    right = get_image_arr(cam="right")

    left, right = rectify_set(left, right)

    left = grayscale(left)
    right = grayscale(right)

    left = resize(left)
    right = resize(right)
    print(left.shape, right.shape)
    left = crop(left, 60, 40, 100,40) #Values are adaptable to  the SCALING (config.py) automatically
    right = crop(right, 60, 40, 100,40) #Values are adaptable to  the SCALING (config.py) automatically
    show_images(left, right)

    disparity_map = get_disparity_map(left, right)
    np.save('disparity_map.npy', disparity_map)
    show_disparity_map(disparity_map) 

    # depth_map = get_depth_map(disparity_map)  
    # color = resize(get_image_arr(cam="left"))
    # color = crop(color, 60, 40, 100, 40) #Values are adaptable to the SCALING (config.py) automatically
    # points = get_points(depth_map, np.array(color))
    
if __name__ == "__main__": 
    main()


    