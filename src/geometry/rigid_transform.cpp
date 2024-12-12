#include "../../include/geometry/rigid_transform.hpp"
#include <iostream>
#include <iomanip>
#include <numeric>
#include <cmath>

namespace slam {

void RigidTransformSolver::testRigidTransform(const Camera& camera) {
    if (camera.photos.size() >= 2) {
        // 计算第一张和第二张照片之间的刚体变换
        RigidTransform transform = camera.calculateRigidTransform(
            camera.photos[0],
            camera.photos[1]
        );

        // 使用第一张照片和变换参数计算第二张照片的位姿
        Photo calculatedPhoto = camera.calculateNewPose(
            camera.photos[0],
            transform
        );

        // 输出详细的比较结果
        std::cout << "\n=== 原始第二张照片参数 ===" << std::endl;
        std::cout << "位置 (X,Y,Z): "
            << camera.photos[1].position.x << ", "
            << camera.photos[1].position.y << ", "
            << camera.photos[1].position.z << std::endl;

        std::cout << "旋转矩阵：" << std::endl;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                std::cout << camera.photos[1].rotation.data[i][j] << " ";
            }
            std::cout << std::endl;
        }

        std::cout << "\n=== 计算得到的第二张照片参数 ===" << std::endl;
        std::cout << "位置 (X,Y,Z): "
            << calculatedPhoto.position.x << ", "
            << calculatedPhoto.position.y << ", "
            << calculatedPhoto.position.z << std::endl;

        std::cout << "旋转矩阵：" << std::endl;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                std::cout << calculatedPhoto.rotation.data[i][j] << " ";
            }
            std::cout << std::endl;
        }

        // 计算误差
        double position_error = sqrt(
            pow(camera.photos[1].position.x - calculatedPhoto.position.x, 2) +
            pow(camera.photos[1].position.y - calculatedPhoto.position.y, 2) +
            pow(camera.photos[1].position.z - calculatedPhoto.position.z, 2)
        );

        std::cout << "\n=== 误差分析 ===" << std::endl;
        std::cout << "位置误差（欧氏距离）: " << position_error << std::endl;
    }
}

void RigidTransformSolver::calculateRelativePose(const Photo& photo1,
                                               const Photo& photo2,
                                               Eigen::Matrix3d& R,
                                               Eigen::Vector3d& t) {
    // 将Mat_R转换为Eigen::Matrix3d
    Eigen::Matrix3d R1, R2;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R1(i, j) = photo1.rotation.data[i][j];
            R2(i, j) = photo2.rotation.data[i][j];
        }
    }

    // 计算相对旋转: R = R2 * R1^T
    R = R2 * R1.transpose();

    // 计算相对平移: t = t2 - R * t1
    Eigen::Vector3d t1(photo1.position.x, photo1.position.y, photo1.position.z);
    Eigen::Vector3d t2(photo2.position.x, photo2.position.y, photo2.position.z);
    t = t2 - R * t1;
}

double RigidTransformSolver::validateTransform(const Photo& photo1,
                                             const Photo& photo2,
                                             const Eigen::Matrix3d& R,
                                             const Eigen::Vector3d& t) {
    // 使用计算的变换参数重建第二张照片的位置
    Eigen::Vector3d t1(photo1.position.x, photo1.position.y, photo1.position.z);
    Eigen::Vector3d t2(photo2.position.x, photo2.position.y, photo2.position.z);
    
    // 使用变换参数计算的位置
    Eigen::Vector3d calculated_t2 = R * t1 + t;
    
    // 计算位置误差
    double pos_error = (calculated_t2 - t2).norm();
    
    // 计算旋转误差
    Eigen::Matrix3d R1, R2;
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            R1(i,j) = photo1.rotation.data[i][j];
            R2(i,j) = photo2.rotation.data[i][j];
        }
    }
    
    // 计算旋转误差（使用Frobenius范数）
    double rot_error = (R2 - R * R1).norm();
    
    // 返回综合误差
    return pos_error + rot_error;
}

void RigidTransformSolver::calculateAbsolutePose(
    const Photo& referencePhoto,
    const Eigen::Matrix3d& relativeR,
    const Eigen::Vector3d& relativeT,
    Photo& targetPhoto
) {
    // 计算目标照片的旋转矩阵
    Eigen::Matrix3d R_ref;
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            R_ref(i,j) = referencePhoto.rotation.data[i][j];
        }
    }
    
    // 计算目标照片的旋转矩阵
    Eigen::Matrix3d R_target = relativeR * R_ref;
    
    // 计算目标照片的位置
    Eigen::Vector3d pos_ref(
        referencePhoto.position.x,
        referencePhoto.position.y,
        referencePhoto.position.z
    );
    
    // 修正位置计算: t2 = R * t1 + t
    Eigen::Vector3d pos_target = relativeR * pos_ref + relativeT;

    // 填充结果
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            targetPhoto.rotation.data[i][j] = R_target(i,j);
        }
    }
    targetPhoto.position.x = pos_target(0);
    targetPhoto.position.y = pos_target(1);
    targetPhoto.position.z = pos_target(2);
}
} // namespace slam