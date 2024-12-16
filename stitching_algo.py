from processes.utils import get_image_arr
from processes.disparity_gen import get_disparity_map, show_disparity_map
import numpy as np
import cv2
import glob

def normalize_disparity(disparity_map):
    return cv2.normalize(disparity_map, None, 
                         alpha=0, beta=255, 
                         norm_type=cv2.NORM_MINMAX, 
                         dtype=cv2.CV_8U)

left_img1 = get_image_arr("synthetic2/1", "im0", greyscale=True)
right_img1 = get_image_arr("synthetic2/1", "im1", greyscale=True)

left_img2 = get_image_arr("synthetic2/2", "im0", greyscale=True)
right_img2 = get_image_arr("synthetic2/2", "im1_0", greyscale=True)

disparity1 = get_disparity_map(left_img1, right_img1, 8)
disparity2 = get_disparity_map(left_img2, right_img2, 8)

disp1 = normalize_disparity(disparity1)
disp2 = normalize_disparity(disparity2)
disparaties = [disp1, disp2]

edged = cv2.Canny(disp1, 30, 200) 

contours, hierarchy = cv2.findContours(edged,  
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 
  
cv2.imshow('Canny Edges After Contouring', edged) 
cv2.waitKey(0) 
  
print("Number of Contours found = " + str(len(contours))) 
  
# Draw all contours 
# -1 signifies drawing all contours 
cv2.drawContours(disp1, contours, -1, (0, 255, 0), 3) 
  
cv2.imshow('Contours', disp1) 
cv2.waitKey(0) 
cv2.destroyAllWindows() 