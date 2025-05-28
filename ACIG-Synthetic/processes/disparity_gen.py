import cv2
import numpy as np
import matplotlib.pyplot as plt

def get_disparity_map(left_image, right_image, multiplier, block_size):
    stereo = cv2.StereoSGBM_create(numDisparities=16 * multiplier, blockSize=block_size)
    disparity = stereo.compute(left_image, right_image).astype(np.float32)
    disparity = filter_zeros(disparity)
    return disparity

def filter_zeros(disparity_map):
    # Create a binary mask for non-zero values
    valid_mask = disparity_map > np.average(disparity_map)

    filtered_disparity = disparity_map.copy()
    filtered_disparity[~valid_mask] = np.nan  # Set invalid regions to NaN

    filtered_disparity = filtered_disparity - np.nanmin(filtered_disparity)
    filtered_disparity = np.nan_to_num(filtered_disparity, nan=0.0)

    return filtered_disparity

def show_disparity_map(disparity_map, title="Disparity Map"):
    plt.imshow(disparity_map, cmap='plasma')
    plt.colorbar()
    plt.title(title)
    plt.show()




