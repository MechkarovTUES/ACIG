from PIL import Image, ImageFilter
from .config import ID, JSON, K1, D1, K2, D2, R, T, SCALE
import numpy as np
import os, json, math
import cv2
import scipy.io

def get_image_arr(cam = "left"):
    data = load_json(JSON)
    img = cv2.imread(f"{data[ID][cam]}")
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    return img

def load_json(json_file_path):

    if not os.path.exists(json_file_path):
        raise FileNotFoundError(f"JSON file not found: {json_file_path}")

    with open(json_file_path, 'r') as file:
        data = json.load(file)

    result = []

    for entry in data:
        left = entry.get('left')
        right = entry.get('right')
        result.append({"left": left, "right": right})

    return result

def rectify_set(imgL, imgR):
    image_size = (imgL.shape[1], imgL.shape[0])

    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
    K1, D1, K2, D2, image_size, R, T, flags=cv2.CALIB_ZERO_DISPARITY)
    # Undistort + rectify maps
    map1x, map1y = cv2.initUndistortRectifyMap(K1, D1, R1, P1, image_size, cv2.CV_32FC1)
    map2x, map2y = cv2.initUndistortRectifyMap(K2, D2, R2, P2, image_size, cv2.CV_32FC1)

    # Apply rectification
    rectL = cv2.remap(imgL, map1x, map1y, cv2.INTER_LINEAR)
    rectR = cv2.remap(imgR, map2x, map2y, cv2.INTER_LINEAR)

    return rectL, rectR

def crop(img, x1, x2, y1, y2):
    """
    Crops the images to the specified coordinates.
    
    Parameters:
    imgL: Left image
    imgR: Right image
    [x1, x2, y1, y2]: Coordinates for cropping in the format [x1, x2, y1, y2]
    
    Returns:
    Cropped left and right images.
    """
    h, w = img.shape[:2] #both images are the same size
    cropped = img[int(y1*SCALE):int(h - (y2*SCALE)), int(x1*SCALE): int(w - (x2*SCALE))]
    # croppedR = imgR[int(y1*SCALE):int(h - (y2*SCALE)), int(x1*SCALE): int(w - (x2*SCALE))]
    
    return cropped

def show_images(imgL, imgR):
    cv2.imshow("Rectified Left Image", imgL)
    cv2.imshow("Rectified Right Image", imgR)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def grayscale(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return gray

def resize(image):
    resized = cv2.resize(image, (0, 0), fx=SCALE, fy=SCALE)
    return resized


