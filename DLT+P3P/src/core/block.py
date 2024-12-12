import xml.etree.ElementTree as ET
import numpy as np
from typing import List, Tuple, Optional
from ..structures.camera import Camera
from ..structures.matrices import Mat_K, Mat_R
from ..structures.vectors import Vec_t
from ..structures.features import TiePnt, Color, Obser
from ..structures.photo import Photo
from ..utils.transformations import rotation_matrix_to_euler_angles
from ..utils.pose_estimation import (
    dlt_pose_estimation, 
    p3p_pose_estimation, 
    ransac_pose_estimation,
    undistort_points
)

class Block:
    def __init__(self):
        self.camera = Camera(Mat_K())
        self.tiepoints: List[TiePnt] = []
        
    def load_from_xml(self, filename: str) -> bool:
        try:
            tree = ET.parse(filename)
            root = tree.getroot()
            block_elem = root.find('Block')
            if block_elem is None:
                return False
                
            # 读取相机参数
            photogroups = block_elem.find('Photogroups')
            if photogroups is not None:
                photogroup = photogroups.find('Photogroup')
                if photogroup is not None:
                    # 读取内参
                    f = float(photogroup.find('FocalLength').text)
                    pp = photogroup.find('PrincipalPoint')
                    cx = float(pp.find('x').text)
                    cy = float(pp.find('y').text)
                    
                    # 设置相机内参矩阵
                    self.camera.intrinsic = Mat_K(np.array([
                        [f, 0, cx],
                        [0, f, cy],
                        [0, 0, 1]
                    ]))
                    
                    # 读取畸变参数
                    distortion = photogroup.find('Distortion')
                    if distortion is not None:
                        self.camera.k1 = float(distortion.find('K1').text)
                        self.camera.k2 = float(distortion.find('K2').text)
                        self.camera.p1 = float(distortion.find('P1').text)
                        self.camera.p2 = float(distortion.find('P2').text)
                    
                    # 读取照片信息
                    for photo in photogroup.findall('Photo'):
                        photo_data = self._parse_photo(photo)
                        self.camera.photos.append(photo_data)
            
            # 读取特征点
            tiepoints_elem = block_elem.find('TiePoints')
            if tiepoints_elem is not None:
                for tiepoint in tiepoints_elem.findall('TiePoint'):
                    tp = self._parse_tiepoint(tiepoint)
                    self.tiepoints.append(tp)
                    
            return True
            
        except Exception as e:
            print(f"加载XML文件失败: {str(e)}")
            return False
    
    def _parse_photo(self, photo_elem) -> Photo:
        # 解析照片信息的辅助方法
        image_path = photo_elem.find('ImagePath').text
        label = image_path.split('/')[-1]
        
        pose = photo_elem.find('Pose')
        center = pose.find('Center')
        position = Vec_t(
            float(center.find('x').text),
            float(center.find('y').text),
            float(center.find('z').text)
        )
        
        rotation = pose.find('Rotation')
        rot_matrix = np.zeros((3, 3))
        for i in range(3):
            for j in range(3):
                elem = rotation.find(f'M_{i}{j}')
                rot_matrix[i][j] = float(elem.text)
                
        # 使用工具函数计算欧拉角
        euler_angles = rotation_matrix_to_euler_angles(rot_matrix)
        
        return Photo(
            label=label,
            position=position,
            rotation=Mat_R(rot_matrix),
            euler_angles=euler_angles
        ) 
    
    def _parse_tiepoint(self, tiepoint_elem) -> TiePnt:
        """解析特征点信息"""
        position = tiepoint_elem.find('Position')
        pos = Vec_t(
            float(position.find('x').text),
            float(position.find('y').text),
            float(position.find('z').text)
        )
        
        color_elem = tiepoint_elem.find('Color')
        color = None
        if color_elem is not None:
            # 将0-1范围的浮点数转换为0-255范围的整数
            r = int(float(color_elem.find('Red').text) * 255)
            g = int(float(color_elem.find('Green').text) * 255)
            b = int(float(color_elem.find('Blue').text) * 255)
            color = Color(r, g, b)
            
        tp = TiePnt(position=pos, color=color)
        
        # 解析观测点
        for measurement in tiepoint_elem.findall('Measurement'):
            photo_id = int(measurement.find('PhotoId').text)
            x = float(measurement.find('x').text)
            y = float(measurement.find('y').text)
            tp.observations.append(Obser(photo_id, x, y))
            
        return tp

    def get_corresponding_points(self, photo_id: int) -> Tuple[List[Vec_t], List[Obser]]:
        """获取指定照片的3D点和对应的2D观测点"""
        points_3d = []
        points_2d = []
        
        for tp in self.tiepoints:
            for obs in tp.observations:
                if obs.photo_id == photo_id:
                    points_3d.append(tp.position)
                    points_2d.append(obs)
                    break
                    
        return points_3d, points_2d

    def estimate_pose_dlt(self, photo_id: int) -> Optional[Tuple[Mat_R, Vec_t, float]]:
        """使用DLT估计相机位姿"""
        points_3d, points_2d = self.get_corresponding_points(photo_id)
        if len(points_3d) < 6:
            print(f"点数不足，需要至少6个点，当前只有{len(points_3d)}个点")
            return None
            
        # 获取相机参数
        K = self.camera.intrinsic.data
        
        # 使用原始点进行DLT估计
        print(f"使用{len(points_3d)}个点进行DLT估计")
        return dlt_pose_estimation(points_3d, points_2d, K)

    def estimate_pose_p3p(self, photo_id: int) -> Tuple[Optional[Mat_R], Optional[Vec_t]]:
        """使用P3P估计相机位姿"""
        points_3d, points_2d = self.get_corresponding_points(photo_id)
        if len(points_3d) < 4:  # P3P需要至少4个点
            print(f"点数不足，需要至少4个点，当前只有{len(points_3d)}个点")
            return None, None

        # 取相机参数
        K = self.camera.intrinsic.data
        
        return p3p_pose_estimation(points_3d, points_2d, K)

    def estimate_pose_ransac(self, photo_id: int, method: str = 'dlt') -> Optional[Tuple[Mat_R, Vec_t, List[bool], float]]:
        """使用RANSAC估计相机位姿"""
        points_3d, points_2d = self.get_corresponding_points(photo_id)
        
        if len(points_3d) < 6:
            print(f"点数不足，需要至少6个点，当前只有{len(points_3d)}个点")
            return None
        
        # 获取相机参数
        K = self.camera.intrinsic.data
        
        # 调用RANSAC位姿估计
        print(f"使用{len(points_3d)}个点进行RANSAC {method}估计")
        return ransac_pose_estimation(
            points_3d=points_3d,
            points_2d=points_2d,
            K=K,
            method=method,
            ransac_iterations=200,  # 增加迭代次数
            ransac_threshold=500.0  # 设置更大的阈值
        )