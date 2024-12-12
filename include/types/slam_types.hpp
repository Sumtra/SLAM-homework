#ifndef SLAM_TYPES_HPP
#define SLAM_TYPES_HPP

#include <Eigen/Dense>
#include <vector>
#include "common_types.hpp"
#include <tinyxml2.h>

namespace slam {
    // 前向声明
    class Camera;
    class Block;

    // 本质矩阵计算结果的结构体
    struct EssentialMatrixResult {
        Eigen::Matrix3d E;
        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        std::vector<bool> inliers;
        double score;
    };
    struct RigidTransform {
        Eigen::Matrix3d R;  // 旋转矩阵
        Eigen::Vector3d t;  // 平移向量
    };
    // 辅助函数声明
    Vec_t rotationMatrixToEulerAngles(double r00, double r01, double r02,
        double r10, double r11, double r12,
        double r20, double r21, double r22);
        

} // namespace slam

#endif // SLAM_TYPES_HPP 