import numpy as np
from ..structures.vectors import Vec_t

def rotation_matrix_to_euler_angles(R: np.ndarray) -> Vec_t:
    """旋转矩阵转欧拉角"""
    # ... 旋转矩阵转欧拉角代码 ...

def recover_scale(R: np.ndarray, t: np.ndarray, points_3d: np.ndarray) -> float:
    """使用3D点的距离比例恢复尺度"""
    # 计算3D点之间的距离
    dist_3d = np.linalg.norm(points_3d[1:] - points_3d[:-1], axis=1)
    # 计算变换后点之间的距离
    points_transformed = (R @ points_3d.T).T + t
    dist_transformed = np.linalg.norm(points_transformed[1:] - points_transformed[:-1], axis=1)
    # 计算尺度比例
    return np.median(dist_3d / dist_transformed) 