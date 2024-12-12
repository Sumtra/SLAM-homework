import numpy as np
import cv2
from typing import List, Tuple, Optional
from ..structures.vectors import Vec_t
from ..structures.matrices import Mat_R
from ..structures.features import Obser

def compute_reprojection_error(points_3d: np.ndarray, 
                             points_2d: List[Obser], 
                             R: np.ndarray, 
                             t: np.ndarray, 
                             K: np.ndarray) -> Tuple[float, np.ndarray]:
    """计算重投影误差"""
    errors = []
    valid_count = 0
    total_error = 0.0
    
    # 一次性计算所有点的投影
    points_cam = (R @ points_3d.T).T + t  # 转换到相机坐标系
    
    # 检查点是否在相机前方
    valid_mask = points_cam[:, 2] > 0
    if not np.any(valid_mask):
        return float('inf'), np.full(len(points_3d), float('inf'))
    
    # 投影到图像平面
    points_proj = points_cam / points_cam[:, 2:]  # 归一化
    points_proj = (K @ points_proj.T).T  # 应用相机内参
    
    # 获取对应的2D观测点
    points_2d_np = np.array([[p.x, p.y] for p in points_2d])
    
    # 计算重投影误差
    errors = np.linalg.norm(points_proj[:, :2] - points_2d_np, axis=1)
    
    # 处理无效点
    errors[~valid_mask] = float('inf')
    
    # 计算平均误差（只考虑有效点）
    valid_errors = errors[valid_mask]
    mean_error = np.mean(valid_errors) if len(valid_errors) > 0 else float('inf')
    
    return mean_error, errors

def undistort_points(points_2d: List[Obser], K: np.ndarray, dist_coeffs: np.ndarray) -> np.ndarray:
    """对图像点进行畸变校正"""
    points = np.array([[p.x, p.y] for p in points_2d], dtype=np.float32)
    undistorted = cv2.undistortPoints(points, K, dist_coeffs, P=K)
    return undistorted.reshape(-1, 2)

def dlt_pose_estimation(points_3d: List[Vec_t], points_2d: List[Obser], K: np.ndarray) -> Tuple[Mat_R, Vec_t, float]:
    """DLT算法实现"""
    if len(points_3d) < 6:
        print(f"点数不足，需要至少6个点，当前只有{len(points_3d)}个点")
        return Mat_R(), Vec_t(0, 0, 0), float('inf')

    try:
        # 转换点为numpy数组
        points_3d_np = np.array([[p.x, p.y, p.z] for p in points_3d])
        points_2d_np = np.array([[p.x, p.y] for p in points_2d])

        # 构建DLT方程组
        A = np.zeros((2 * len(points_3d), 12))
        for i in range(len(points_3d)):
            X, Y, Z = points_3d_np[i]
            u, v = points_2d_np[i]
            A[2*i] = [X, Y, Z, 1, 0, 0, 0, 0, -u*X, -u*Y, -u*Z, -u]
            A[2*i+1] = [0, 0, 0, 0, X, Y, Z, 1, -v*X, -v*Y, -v*Z, -v]

        # 使用SVD求解
        _, _, Vt = np.linalg.svd(A)
        P = Vt[-1].reshape(3, 4)

        # 从投影矩阵中提取R和t
        P = P / np.linalg.norm(P[:3, :3])
        R = P[:3, :3]
        t = P[:3, 3]

        # 确保R是正交矩阵
        U, _, Vt = np.linalg.svd(R)
        R = U @ Vt

        # 确保行列式为1
        if np.linalg.det(R) < 0:
            R = -R
            t = -t

        return Mat_R(R), Vec_t(*t), 0.0  # 暂时不计算重投影误差

    except Exception as e:
        print(f"DLT算法出错: {str(e)}")
        return Mat_R(), Vec_t(0, 0, 0), float('inf')

def p3p_pose_estimation(points_3d: List[Vec_t], points_2d: List[Obser], K: np.ndarray) -> Tuple[Optional[Mat_R], Optional[Vec_t], float]:
    """P3P算法实现"""
    if len(points_3d) < 4:
        print(f"点数不足，需要至少4个点，当前只有{len(points_3d)}个点")
        return Mat_R(), Vec_t(0, 0, 0), float('inf')

    # 转换点为numpy数组
    points_3d_np = np.array([[p.x, p.y, p.z] for p in points_3d], dtype=np.float32)
    points_2d_np = np.array([[p.x, p.y] for p in points_2d], dtype=np.float32)

    try:
        print(f"使用{len(points_3d)}个点进行P3P估计")

        # 归一化2D点
        points_2d_norm = np.zeros_like(points_2d_np)
        points_2d_norm[:, 0] = (points_2d_np[:, 0] - K[0, 2]) / K[0, 0]
        points_2d_norm[:, 1] = (points_2d_np[:, 1] - K[1, 2]) / K[1, 1]

        # 使用归一化的2D点进行PnP求解
        success, rvec, tvec = cv2.solvePnP(
            points_3d_np,
            points_2d_norm,
            np.eye(3),  # 使用单位矩阵作为相机矩阵
            None,  # 不使用畸变系数
            flags=cv2.SOLVEPNP_ITERATIVE  # 使用迭代法
        )

        if not success:
            return Mat_R(), Vec_t(0, 0, 0), float('inf')

        # 将旋转向量转换为旋转矩阵
        R, _ = cv2.Rodrigues(rvec)
        t = tvec.flatten()

        # 计算重投影误差
        mean_error, errors = compute_reprojection_error(points_3d_np, points_2d, R, t, K)
        if mean_error < float('inf'):
            print(f"P3P平均重投影误差: {mean_error:.3f}像素")

        return Mat_R(R), Vec_t(*t), mean_error

    except Exception as e:
        print(f"P3P算法出错: {str(e)}")
        return Mat_R(), Vec_t(0, 0, 0), float('inf')

def ransac_pose_estimation(points_3d: List[Vec_t], 
                         points_2d: List[Obser], 
                         K: np.ndarray,
                         method: str = 'dlt',
                         ransac_iterations: int = 200,
                         ransac_threshold: float = 500.0) -> Tuple[Mat_R, Vec_t, List[bool], float]:
    """RANSAC + DLT/P3P 位姿估计"""
    print(f"开始RANSAC {method}位姿估计...")
    print(f"点对数量: {len(points_3d)}")
    
    points_3d_np = np.array([[p.x, p.y, p.z] for p in points_3d])
    points_2d_np = np.array([[p.x, p.y] for p in points_2d])

    try:
        if method == 'p3p':
            min_points = 4  # P3P需要4个点
            if len(points_3d) < min_points:
                raise Exception(f"点数不足，需要至少{min_points}个点，当前只有{len(points_3d)}个点")

            best_R = None
            best_t = None
            best_inliers = 0
            best_inliers_mask = [False] * len(points_3d)
            best_error = float('inf')

            print(f"开始RANSAC迭代 ({ransac_iterations}次)...")
            successful_iterations = 0

            for iter_count in range(ransac_iterations):
                try:
                    # 随机选择4个点
                    indices = np.random.choice(len(points_3d), min_points, replace=False)
                    sample_3d = [points_3d[i] for i in indices]
                    sample_2d = [points_2d[i] for i in indices]

                    # 使用P3P估计位姿
                    R_temp, t_temp, error_temp = p3p_pose_estimation(sample_3d, sample_2d, K)
                    
                    if error_temp == float('inf') or R_temp is None or t_temp is None:
                        continue

                    successful_iterations += 1

                    # 计算所有点的重投影误差
                    mean_error, errors = compute_reprojection_error(
                        points_3d_np, 
                        points_2d, 
                        R_temp.data, 
                        np.array([t_temp.x, t_temp.y, t_temp.z]), 
                        K
                    )

                    # 统计内点
                    current_mask = [error < ransac_threshold for error in errors]
                    inliers_count = sum(current_mask)

                    print(f"迭代 {iter_count + 1}: 内点数: {inliers_count}, 误差: {mean_error:.3f}")

                    # 更新最佳模型
                    if inliers_count > best_inliers or (inliers_count == best_inliers and mean_error < best_error):
                        best_inliers = inliers_count
                        best_R = R_temp
                        best_t = t_temp
                        best_inliers_mask = current_mask
                        best_error = mean_error
                        print(f"找到更好的模型！内点: {inliers_count}, 误差: {mean_error:.3f}")

                except Exception as e:
                    print(f"迭代 {iter_count + 1} 失败: {str(e)}")
                    continue

            print(f"成功完成的迭代: {successful_iterations}/{ransac_iterations}")

            if best_R is not None and best_inliers >= min_points:
                # 使用所有内点重新估计位姿
                inlier_3d = [p for p, is_inlier in zip(points_3d, best_inliers_mask) if is_inlier]
                inlier_2d = [p for p, is_inlier in zip(points_2d, best_inliers_mask) if is_inlier]
                
                print(f"使用{len(inlier_3d)}个内点进行最终估计...")
                final_R, final_t, final_error = p3p_pose_estimation(inlier_3d, inlier_2d, K)
                print(f"RANSAC最终结果: {best_inliers}/{len(points_3d)}个内点, 误差: {final_error:.3f}像素")
                return final_R, final_t, best_inliers_mask, final_error

            raise Exception(f"RANSAC未能找到足够的内点 (最佳内点数: {best_inliers})")

        else:  # DLT方法
            min_points = 6
            if len(points_3d) < min_points:
                raise Exception(f"点数不足，需要至少{min_points}个点，当前只有{len(points_3d)}个点")
                
            best_R = None
            best_t = None
            best_inliers = 0
            best_inliers_mask = [False] * len(points_3d)
            best_error = float('inf')

            print(f"开始RANSAC迭代 ({ransac_iterations}次)...")
            successful_iterations = 0

            for iter_count in range(ransac_iterations):
                try:
                    # 随机选择最小点集
                    indices = np.random.choice(len(points_3d), min_points, replace=False)
                    sample_3d = [points_3d[i] for i in indices]
                    sample_2d = [points_2d[i] for i in indices]

                    # 使用DLT估计位姿
                    R_temp, t_temp, error_temp = dlt_pose_estimation(sample_3d, sample_2d, K)
                    
                    if error_temp == float('inf'):
                        continue

                    successful_iterations += 1

                    # 计算所有点的重投影误差
                    mean_error, errors = compute_reprojection_error(
                        points_3d_np, 
                        points_2d, 
                        R_temp.data, 
                        np.array([t_temp.x, t_temp.y, t_temp.z]), 
                        K
                    )

                    # 统计内点
                    current_mask = [error < ransac_threshold for error in errors]
                    inliers_count = sum(current_mask)

                    print(f"迭代 {iter_count + 1}: 内点数: {inliers_count}, 误差: {mean_error:.3f}")

                    # 更新最佳模型
                    if inliers_count > best_inliers or (inliers_count == best_inliers and mean_error < best_error):
                        best_inliers = inliers_count
                        best_R = R_temp
                        best_t = t_temp
                        best_inliers_mask = current_mask
                        best_error = mean_error
                        print(f"找到更好的模型！内点: {inliers_count}, 误差: {mean_error:.3f}")

                except Exception as e:
                    print(f"迭代 {iter_count + 1} 失败: {str(e)}")
                    continue

            print(f"成功完成的迭代: {successful_iterations}/{ransac_iterations}")

            if best_R is not None and best_inliers >= min_points:
                # 使用所有内点重新估计位姿
                inlier_3d = [p for p, is_inlier in zip(points_3d, best_inliers_mask) if is_inlier]
                inlier_2d = [p for p, is_inlier in zip(points_2d, best_inliers_mask) if is_inlier]
                
                print(f"使用{len(inlier_3d)}个内点进行最终估计...")
                final_R, final_t, final_error = dlt_pose_estimation(inlier_3d, inlier_2d, K)
                print(f"RANSAC最终结果: {best_inliers}/{len(points_3d)}个内点, 误差: {final_error:.3f}像素")
                return final_R, final_t, best_inliers_mask, final_error

            raise Exception(f"RANSAC未能找到足够的内点 (最佳内点数: {best_inliers})")

    except Exception as e:
        print(f"RANSAC算法出错: {str(e)}")
        return Mat_R(), Vec_t(0, 0, 0), [False] * len(points_3d), float('inf') 