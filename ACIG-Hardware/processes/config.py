import scipy.io
# SET SELECTION
# -----------------------------
JSON = "data.json"
ID = 0 #ID numeration starts from 0 | 0 -> Im1.stereo_set
CALIBRATION = "opencv_calib.mat"
SCALE = 0.25 # Resize factor for images

# DISPARITY PARAMETERS
# -----------------------------
DISP = 16 * 4 # Maximum disparity
BLOCK = 19
# CAMERA PARAMETERS
# -----------------------------
data = scipy.io.loadmat(CALIBRATION)
K1 = data['K1']
D1 = data['D1'].flatten()
K2 = data['K2']
D2 = data['D2'].flatten()
R = data['R']
T = data['T'].flatten()







