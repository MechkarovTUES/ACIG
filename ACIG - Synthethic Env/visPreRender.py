import open3d as o3d

# file = "merged_cloud.ply"
file = "cloud_env2.ply"

# Зареждаме готовия облак
source = o3d.io.read_point_cloud(file)

# Визуализация
o3d.visualization.draw_geometries([source])