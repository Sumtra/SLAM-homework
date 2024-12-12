#ifndef ESSENTIAL_MATRIX_HPP
#define ESSENTIAL_MATRIX_HPP

#include <Eigen/Dense>
#include "../geometry/camera.hpp"
#include "../types/slam_types.hpp"

namespace slam {

class Camera;  // 前向声明

class EssentialMatrixSolver {
public:
    // 主要接口函数
    // 修改为与实现一致的声明
    static void computeEssentialMatrix(
        const Camera& camera,
        const std::vector<TiePnt>& tiepoints,
        const int photo1_id,
        const int photo2_id,
        Eigen::Matrix3d& R,
        Eigen::Vector3d& t);

    // 计算本质矩阵
    static Eigen::Matrix3d calculateEssentialMatrix(
        const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>& matches,
        const Eigen::Matrix3d& K);
    // 测试本质矩阵
    static void testEssentialMatrix(
        const Camera& camera,
        const std::vector<TiePnt>& tiepoints);

    // 辅助函数
    static void decomposeEssentialMatrix(
        const Eigen::Matrix3d& E,
        std::vector<Eigen::Matrix3d>& Rs,
        std::vector<Eigen::Vector3d>& ts);

    static void selectBestRT(
        const std::vector<Eigen::Matrix3d>& Rs,
        const std::vector<Eigen::Vector3d>& ts,
        const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>& matches,
        const Eigen::Matrix3d& K,
        Eigen::Matrix3d& bestR,
        Eigen::Vector3d& bestT);

private:
    static void getMatchingPoints(
        const std::vector<TiePnt>& tiepoints,
        int photo1_id,
        int photo2_id,
        std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>& matches);
};

} // namespace slam

#endif // ESSENTIAL_MATRIX_HPP 