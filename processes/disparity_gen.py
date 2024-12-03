import cv2
import numpy as np
import  matplotlib.pyplot as plt

def get_disparity_map(left_image, right_image, multiplier):
    stereo = cv2.StereoSGBM_create(minDisparity=0, numDisparities=16 * multiplier, blockSize=19)
    disparity = stereo.compute(left_image, right_image).astype(np.float32)
    # disparity = disparity / 10
    return disparity

def show_disparity_map(disparity_map):
    plt.imshow(disparity_map, cmap='plasma')
    plt.colorbar()
    plt.show()