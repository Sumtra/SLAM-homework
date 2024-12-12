#include <Eigen/Dense>
#include "../../include/geometry/essential_matrix.hpp"
#include "../../include/types/slam_types.hpp"
#include <iostream>

namespace slam {

void EssentialMatrixSolver::computeEssentialMatrix(
    const Camera& camera,
    const std::vector<TiePnt>& tiepoints,
    const int photo1_id,
    const int photo2_id,
    Eigen::Matrix3d& R,
    Eigen::Vector3d& t) {

    // 获取匹配点对
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> matches;
    getMatchingPoints(tiepoints, photo1_id, photo2_id, matches);

    if (matches.size() < 8) {
        std::cout << "匹配点对不足，无法计算本质矩阵" << std::endl;
        return;
    }

    // 转换内参矩阵为Eigen格式
    Eigen::Matrix3d K_eigen;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            K_eigen(i, j) = camera.intrinsic.data[i][j];
        }
    }

    // 计算本质矩阵
    Eigen::Matrix3d E = calculateEssentialMatrix(matches, K_eigen);

    // 分解本质矩阵得到可能的R和t
    std::vector<Eigen::Matrix3d> Rs;
    std::vector<Eigen::Vector3d> ts;
    decomposeEssentialMatrix(E, Rs, ts);

    // 选择最佳的R和t
    selectBestRT(Rs, ts, matches, K_eigen, R, t);
}

void EssentialMatrixSolver::getMatchingPoints(
    const std::vector<TiePnt>& tiepoints,
    const int photo1_id,
    const int photo2_id,
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>& matches) {

    matches.clear();
    for (const auto& tp : tiepoints) {
        Eigen::Vector2d pt1, pt2;
        bool found1 = false, found2 = false;

        for (const auto& obs : tp.observations) {
            if (obs.photoId == photo1_id) {
                pt1 = Eigen::Vector2d(obs.x, obs.y);
                found1 = true;
            }
            if (obs.photoId == photo2_id) {
                pt2 = Eigen::Vector2d(obs.x, obs.y);
                found2 = true;
            }
        }

        if (found1 && found2) {
            matches.push_back({ pt1, pt2 });
        }
    }
}

Eigen::Matrix3d EssentialMatrixSolver::calculateEssentialMatrix(
    const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>& matches,
    const Eigen::Matrix3d& K) {

    // 构建系数矩阵A
    Eigen::MatrixXd A(matches.size(), 9);

    for (size_t i = 0; i < matches.size(); i++) {
        // 归一化坐标
        Eigen::Vector3d p1 = K.inverse() * Eigen::Vector3d(matches[i].first.x(), matches[i].first.y(), 1.0);
        Eigen::Vector3d p2 = K.inverse() * Eigen::Vector3d(matches[i].second.x(), matches[i].second.y(), 1.0);

        A(i, 0) = p2.x() * p1.x();
        A(i, 1) = p2.x() * p1.y();
        A(i, 2) = p2.x() * p1.z();
        A(i, 3) = p2.y() * p1.x();
        A(i, 4) = p2.y() * p1.y();
        A(i, 5) = p2.y() * p1.z();
        A(i, 6) = p2.z() * p1.x();
        A(i, 7) = p2.z() * p1.y();
        A(i, 8) = p2.z() * p1.z();
    }

    // SVD分解
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
    Eigen::VectorXd e = svd.matrixV().col(8);

    // 重构本质矩阵
    Eigen::Matrix3d E;
    E << e(0), e(1), e(2),
         e(3), e(4), e(5),
         e(6), e(7), e(8);

    // 强制奇异值为[1,1,0]
    Eigen::JacobiSVD<Eigen::Matrix3d> svd_E(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector3d singular_values(1, 1, 0);
    E = svd_E.matrixU() * singular_values.asDiagonal() * svd_E.matrixV().transpose();

    return E;
}

void EssentialMatrixSolver::decomposeEssentialMatrix(
    const Eigen::Matrix3d& E,
    std::vector<Eigen::Matrix3d>& Rs,
    std::vector<Eigen::Vector3d>& ts) {

    // 1. SVD分解本质矩阵
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    // 2. 构造W矩阵
    Eigen::Matrix3d W;
    W << 0, -1, 0,
         1,  0, 0,
         0,  0, 1;

    // 3. 计算四种可能的R和t组合
    Rs.clear();
    ts.clear();

    Rs.push_back(U * W * V.transpose());
    Rs.push_back(U * W * V.transpose());
    Rs.push_back(U * W.transpose() * V.transpose());
    Rs.push_back(U * W.transpose() * V.transpose());

    Eigen::Vector3d t = U.col(2);
    ts.push_back(t);
    ts.push_back(-t);
    ts.push_back(t);
    ts.push_back(-t);

    // 4. 确保R是正交矩阵且行列式为1
    for (auto& R : Rs) {
        if (R.determinant() < 0) {
            R = -R;
        }
    }
}

void EssentialMatrixSolver::selectBestRT(
    const std::vector<Eigen::Matrix3d>& Rs,
    const std::vector<Eigen::Vector3d>& ts,
    const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>& matches,
    const Eigen::Matrix3d& K,
    Eigen::Matrix3d& best_R,
    Eigen::Vector3d& best_t) {

    int best_score = 0;
    int best_idx = 0;

    // 对每个可能的R,t组合进行测试
    for (size_t i = 0; i < Rs.size(); i++) {
        const Eigen::Matrix3d& R = Rs[i];
        const Eigen::Vector3d& t = ts[i];
        int score = 0;

        // 对每个匹配点对进行测试
        for (const auto& match : matches) {
            // 三角化点
            Eigen::Vector3d p1(match.first.x(), match.first.y(), 1.0);
            Eigen::Vector3d p2(match.second.x(), match.second.y(), 1.0);

            p1 = K.inverse() * p1;
            p2 = K.inverse() * p2;

            // 检查点是否在两个相机前面
            if (p1.z() > 0 && (R * p1 + t).z() > 0) {
                score++;
            }
        }

        // 更新最佳结果
        if (score > best_score) {
            best_score = score;
            best_idx = i;
        }
    }

    // 设置最佳的R和t
    best_R = Rs[best_idx];
    best_t = ts[best_idx];
}

void EssentialMatrixSolver::testEssentialMatrix(
    const Camera& camera,
    const std::vector<TiePnt>& tiepoints) {

    std::cout << "\n=== 本质矩阵分解结果 ===" << std::endl;

    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    computeEssentialMatrix(camera, tiepoints, 0, 1, R, t);

    std::cout << "估计的旋转矩阵 R：" << std::endl << R << std::endl;
    std::cout << "估计的平移向量 t：" << std::endl << t.transpose() << std::endl;
}

} // namespace slam