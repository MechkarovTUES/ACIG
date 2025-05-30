import open3d as o3d
import numpy as np
from processes.utils import load_point_clouds, picked_points, \
registration, save_pointcloud

initRegistration = True # Set to True if you want to perform registration

num = 3
pcds = load_point_clouds(voxel_size=1.0, num=7) # num is the number of point clouds to load from the TXT_format dir

if len(pcds) == 0:
    raise ValueError("No point clouds loaded. Please check the input files.")
elif initRegistration:
    source = registration(pcds)
else:
    source = pcds[num]

def geo_reference(pcd):
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # Wait for user interaction
    vis.destroy_window()
    return vis.get_picked_points()

save_pointcloud(source, filename="ACIG_exmpl1")

picked_indices = geo_reference(source)
picked_points(picked_indices, source)
