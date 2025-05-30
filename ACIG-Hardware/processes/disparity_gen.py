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
    #remove invalid values to become nan
    disparity[disparity <= 0] = np.nan
    # disparity = refine_disparity(disparity, left_image)
    # disparity = filter(disparity)
    return disparity

def filter(disparity_map):
    valid_mask = (disparity_map > 10)

    median = np.median(disparity_map[valid_mask])
    disparity_map[~valid_mask] = median

    disparity_filtered = cv2.medianBlur(disparity_map.astype(np.float32), 5)

    disparity_filtered = cv2.bilateralFilter(disparity_filtered.astype(np.float32), d=2, sigmaColor=105, sigmaSpace=205)

    return disparity_filtered

def refine_disparity(disp, left_image):
    """
    disp: float32 disparity in pixels, with invalid ≤0
    returns: float32 refined disparity
    """
    h, w = disp.shape

    # 1) remove small speckles of “bad” disparity
    #    first convert to 16-bit signed if not already
    disp16 = np.int16(disp * 16.0)
    #    filterSpeckles params: (disp, disp_min, maxSpeckleSize, maxDiff)
    cv2.filterSpeckles(disp16, 0, maxSpeckleSize=200, maxDiff=1.0)
    disp = disp16.astype(np.float32) / 16.0

    # 2) build invalid mask
    invalid = (disp <= 10)

    # 3) scan-line fill: for each row, do a 1D interp of invalid pixels
    for y in range(h):
        row = disp[y]
        inv = invalid[y]
        if inv.all():
            # nothing valid on this row → skip
            continue
        valid_x = np.where(~inv)[0]
        valid_v = row[valid_x]
        interp = np.interp(np.arange(w), valid_x, valid_v)
        row[inv] = interp[inv]
        disp[y] = row

    # 4) optional: median + bilateral to knock down any remaining outliers
    # disp = cv2.medianBlur(disp.astype(np.float32), ksize=5)
    disp = cv2.bilateralFilter(disp, d=5, sigmaColor=50, sigmaSpace=50)

    guide = left_image.astype(np.float32) / 255.0  

    # 4) edge‐aware smoothing  
    r   = 3        # window radius
    eps = 1e-3     # smaller = sharper  
    crisp = guided_filter(guide, disp, r, eps)

    return crisp

def guided_filter(I, P, r, eps):
    """
    I: guide image, CV_32F, single‐channel, normalized to [0,1]
    P: input (disparity), CV_32F
    r: radius of the local window
    eps: regularization (smaller → sharper edges)
    """
    # 1) compute means with boxFilter
    mean_I = cv2.boxFilter(I, cv2.CV_32F, (r,r))
    mean_P = cv2.boxFilter(P, cv2.CV_32F, (r,r))
    corr_I = cv2.boxFilter(I*I, cv2.CV_32F, (r,r))
    corr_IP= cv2.boxFilter(I*P, cv2.CV_32F, (r,r))

    # 2) compute linear coefficients a, b
    var_I  = corr_I - mean_I*mean_I
    cov_IP = corr_IP - mean_I*mean_P
    a = cov_IP / (var_I + eps)
    b = mean_P - a*mean_I

    # 3) mean of coefficients
    mean_a = cv2.boxFilter(a, cv2.CV_32F, (r,r))
    mean_b = cv2.boxFilter(b, cv2.CV_32F, (r,r))

    # 4) output
    return mean_a * I + mean_b

def show_disparity_map(disparity_map, title="Disparity Map"):
    plt.figure()
    plt.imshow(disparity_map, cmap='gray')
    plt.colorbar(label=title)
    plt.title("Stereo Disparity Map")
    plt.axis('off')
    plt.tight_layout()

    # plt.savefig("disparity_matplotlib.png", dpi=300) # Optional: Save the output disparity as .png
    plt.show()




