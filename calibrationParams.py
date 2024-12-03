import numpy as np

# Image resolution and FOV
image_width = 3853  # in pixels
image_height = 1841  # in pixels
FOV = 90  # in degrees
BASELINE = 56.46  # in mm (given)

# Step 1: Calculate the focal length in pixels using the FOV and image resolution
focal_length_pixels = image_width / (2 * np.tan(np.radians(FOV) / 2))

# Step 2: Assume principal points are at the center of the images
PRINCIPLE_X_A = image_width / 2
PRINCIPLE_X_B = image_width / 2
PRINCIPLE_Y = image_height / 2

# Step 3: Calculate DOFFS (usually the difference in the x-coordinate of principal points)
DOFFS = PRINCIPLE_X_B - PRINCIPLE_X_A

# Print results
print(f"Focal Length in pixels: {focal_length_pixels}")
print(f"Principal Point (X, Y) for Left Camera: ({PRINCIPLE_X_A}, {PRINCIPLE_Y})")
print(f"Principal Point (X, Y) for Right Camera: ({PRINCIPLE_X_B}, {PRINCIPLE_Y})")
print(f"DOFFS: {DOFFS}")