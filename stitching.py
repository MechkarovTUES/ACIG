import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import griddata

data = np.loadtxt("generated-cloud.txt")
points = data[:, :3]  #(x, y, z)
colors = data[:, 3:]  #(r, g, b)

# Filter NaN points
valid_mask = np.all(np.isfinite(points), axis=1)
points = points[valid_mask]
colors = colors[valid_mask]

