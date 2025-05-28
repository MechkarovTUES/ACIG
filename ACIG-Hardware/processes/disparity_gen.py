import cv2
import numpy as np
import matplotlib.pyplot as plt
from .config import DISP, BLOCK

def get_disparity_map(left_image, right_image):
    stereo = cv2.StereoSGBM_create(
        # minDisparity=min_disp,
        numDisparities=DISP,
        blockSize=BLOCK,
        P1=8 * 2 * BLOCK**1,
        P2=32 * 2 * BLOCK**1,
        # disp12MaxDiff=1,
        # uniquenessRatio=1,
        speckleWindowSize=100,
        speckleRange=32
    )

    # Compute disparity map
    disparity = stereo.compute(left_image, right_image).astype(np.float32) / 16.0

    # disparity = filter(disparity)
    return disparity

def filter(disparity_map):
    valid_mask = (disparity_map > 0)

    median = np.median(disparity_map[valid_mask])
    disparity_map[~valid_mask] = median

    disparity_filtered = cv2.medianBlur(disparity_map.astype(np.float32), 5)

    disparity_filtered = cv2.bilateralFilter(disparity_filtered.astype(np.float32), d=2, sigmaColor=105, sigmaSpace=205)

    return disparity_filtered

def show_disparity_map(disparity_map, title="Disparity Map"):
    plt.figure()
    plt.imshow(disparity_map, cmap='gray')
    plt.colorbar(label=title)
    plt.title("Stereo Disparity Map")
    plt.axis('off')
    plt.tight_layout()

    # plt.savefig("disparity_matplotlib.png", dpi=300) # Optional: Save the output disparity as .png
    plt.show()




