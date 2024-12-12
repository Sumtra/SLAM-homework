#ifndef RIGID_TRANSFORM_HPP
#define RIGID_TRANSFORM_HPP

#include <Eigen/Dense>
#include "../geometry/camera.hpp"
#include "../types/slam_types.hpp"

namespace slam {

    class RigidTransformSolver {
    public:
        // 测试刚体变换计算
        static void testRigidTransform(const Camera& camera);

        // 计算两张影像之间的相对位姿
        static void calculateRelativePose(const Photo& photo1,
            const Photo& photo2,
            Eigen::Matrix3d& R,
            Eigen::Vector3d& t);

        // 验证计算结果
        static double validateTransform(const Photo& photo1,
            const Photo& photo2,
            const Eigen::Matrix3d& R,
            const Eigen::Vector3d& t);
        // 新增方法：根据参考照片和相对位姿计算目标照片的绝对位姿
        static void calculateAbsolutePose(
            const Photo& referencePhoto,
            const Eigen::Matrix3d& relativeR,
            const Eigen::Vector3d& relativeT,
            Photo& targetPhoto
        );
    };
} // namespace slam

#endif // RIGID_TRANSFORM_HPP 