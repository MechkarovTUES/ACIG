import numpy as np
import pandas as pd

def txt_to_ply(txt_file, ply_file, has_color=False):
    # Load the point cloud data from the TXT file
    data = np.loadtxt(txt_file)
    header = ["x", "y", "z"] + (["red", "green", "blue"] if has_color else [])
    
    # Prepare the PLY header
    num_points = data.shape[0]
    ply_header = f"""ply
format ascii 1.0
element vertex {num_points}
property float x
property float y
property float z
"""
    if has_color:
        ply_header += """property uchar red
property uchar green
property uchar blue
"""
    ply_header += "end_header\n"

    # Save to PLY file
    with open(ply_file, "w") as ply:
        ply.write(ply_header)
        pd.DataFrame(data, columns=header).to_csv(
            ply, sep=" ", index=False, header=False
        )

# Example Usage
txt_file = "generated-cloud.txt"  # Replace with your .txt file path
ply_file = "cloud.ply"   # Replace with desired .ply file path
txt_to_ply(txt_file, ply_file, has_color=True)  # Set has_color=True if file has RGB