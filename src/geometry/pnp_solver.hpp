#pragma once

#include <vector>
#include <Eigen/Dense>

namespace slam {

// 定义标准坐标系统
struct CoordinateSystem {
    // 相机坐标系（标准定义）：
    // - 原点：相机光心
    // - Z轴：指向相机前方（光轴方向）
    // - Y轴：指向相机下方
    // - X轴：指向相机右方（右手系）
    
    // 世界坐标系：
    // - 使用右手系
    // - 尺度：米为单位
    
    // 图像坐标系：
    // - 原点：图像左上角
    // - X轴：向右
    // - Y轴：向下
    // - 单位：像素
    
    enum class Type {
        CAMERA,     // 相机坐标系
        WORLD,      // 世界坐标系
        IMAGE       // 图像坐标系
    };

    static const double STANDARD_DEPTH = 200.0;  // 期望的标准深度值（相机坐标系）
    static const double MIN_DEPTH = 50.0;        // 最小有效深度
    static const double MAX_DEPTH = 1000.0;      // 最大有效深度
};

class PnPSolver {
public:
    // 在头文件中添加
    private:
        static bool checkPointInFrontOfCamera(
            const Point3D& point,
            const Mat_R& R,
            const Vec_t& t
        );

        // 坐标系转换结构
        struct Transform {
            Mat_R rotation;    // 旋转矩阵
            Vec_t translation; // 平移向量
            double scale;      // 尺度因子

            Transform() : scale(1.0) {
                // 初始化为单位变换
                rotation = Mat_R::Identity();
                translation = Vec_t::Zero();
            }
        };

        // 新增：坐标系转换和验证函数
        Transform computeWorldToCamera(const std::vector<Point3D>& points3D);
        bool validateTransform(const Transform& transform);
        void normalizePoints(std::vector<Point3D>& points, Transform& transform);
        bool checkCoordinateSystem(const std::vector<Point3D>& points);
}; 