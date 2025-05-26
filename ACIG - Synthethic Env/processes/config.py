import math
# SET SELECTION
# -----------------------------
JSON = "test.json"
SCALE = 4 # 4:1
ID = 3 #ID numeration starts from 0 | 0 -> Im1.stereo_set

# CAMERA PARAMETERS
# -----------------------------
FOCAL_PX=1000.0
BASELINE=0.50 # meters
FOV = 90
ASPECT_RATIO = 4/3

# Geo-referencing parameteres
# -----------------------------
HEIGHT = 5 # meters
METERS_PER_DEGREE_LAT = 111320
H_FOV = 90  
V_FOV = 2 * math.degrees(
    math.atan( math.tan(math.radians(H_FOV/2)) / ASPECT_RATIO )
)







