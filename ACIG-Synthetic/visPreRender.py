import open3d as o3d
from processes.utils import visualize_point_cloud
from processes.config import PLY_PATH

# file = "ACIG_exmpl1.ply"
file = "ACIG_exmpl2.ply"

# Зареждаме готовия облак
source = o3d.io.read_point_cloud(f"{PLY_PATH}/{file}")

# Визуализация
visualize_point_cloud(source, "Prerendered Point Cloud")