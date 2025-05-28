import math
# SET SELECTION
# -----------------------------
DATASET = "data1"
JSON_PATH = f"datasets/{DATASET}.json"
SCALE = 4 # 4:1
ID = 0 #ID numeration starts from 0 | 0 -> Im1.stereo_set

# CAMERA PARAMETERS
# -----------------------------
FOCAL_PX=1000.0
BASELINE=0.50 # meters
ASPECT_RATIO = 4/3

# Geo-referencing parameteres
# -----------------------------
HEIGHT = 10 # meters
METERS_PER_DEGREE_LAT = 111320
H_FOV = 90  
V_FOV = 2 * math.degrees(
    math.atan( math.tan(math.radians(H_FOV/2)) / ASPECT_RATIO )
)

# Visualization parameters
# -----------------------------
TXT_PATH = f"PCDs/TXT_format"
PLY_PATH = "PCDs/PLY_format"






