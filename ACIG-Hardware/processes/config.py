import scipy.io
# SET SELECTION
# -----------------------------
JSON = "data.json"
ID = 4 #ID numeration starts from 0 | 0 -> Im1.stereo_set
CALIBRATION = "opencv_calib.mat"
SCALE = 0.25 # Resize factor for images

# DISPARITY PARAMETERS
# -----------------------------
DISP = int(16 * 16 * SCALE) # Maximum disparity
BLOCK =21
# CAMERA PARAMETERS
# -----------------------------
data = scipy.io.loadmat(CALIBRATION)
K1 = data['K1']
D1 = data['D1'].flatten()
K2 = data['K2']
D2 = data['D2'].flatten()
R = data['R']
T = data['T'].flatten()

FX_PX = 1683.1946 #* SCALE # Focal length in pixels
BASELINE = 80 # Distance between cameras in mm





