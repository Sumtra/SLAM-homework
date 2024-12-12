#include "../include/types/slam_types.hpp"
#include <iostream>
#include <iomanip>
#include <tinyxml2.h>
#include"../include/geometry/camera.hpp"
#include "../include/geometry/essential_matrix.hpp"
#include "../include/geometry/camera.hpp"
#include "../include/types/block.hpp" 
namespace slam {

// Mat_K构造函数实现
Mat_K::Mat_K() {
    data[0][0] = 1.0; data[0][1] = 0.0; data[0][2] = 0.0;
    data[1][0] = 0.0; data[1][1] = 1.0; data[1][2] = 0.0;
    data[2][0] = 0.0; data[2][1] = 0.0; data[2][2] = 1.0;
}

// Camera类成员函数实现
std::pair<double, double> Camera::pixelToNormalized(double x, double y) const {
    double xn = (x - intrinsic.data[0][2]) / intrinsic.data[0][0];
    double yn = (y - intrinsic.data[1][2]) / intrinsic.data[1][1];
    return std::make_pair(xn, yn);
}

std::pair<double, double> Camera::normalizedToPixel(double xn, double yn) const {
    double x = xn * intrinsic.data[0][0] + intrinsic.data[0][2];
    double y = yn * intrinsic.data[1][1] + intrinsic.data[1][2];
    return std::make_pair(x, y);
}

// 添加畸变计算函数实现
std::pair<double, double> Camera::distortPoint(double xn, double yn) const {
    double r2 = xn * xn + yn * yn;
    double radial = 1 + k1 * r2 + k2 * r2 * r2;

    // 计算切向畸变
    double dx = 2 * p1 * xn * yn + p2 * (r2 + 2 * xn * xn);
    double dy = p1 * (r2 + 2 * yn * yn) + 2 * p2 * xn * yn;

    double xd = xn * radial + dx;
    double yd = yn * radial + dy;

    return std::make_pair(xd, yd);
}

std::pair<double, double> Camera::undistortPoint(double x, double y) const {
    // 首先转换为归一化坐标
    auto norm = pixelToNormalized(x, y);
    double xn = norm.first;
    double yn = norm.second;

    // 使用迭代法求解无畸变坐标
    for (int i = 0; i < 5; i++) {
        double r2 = xn * xn + yn * yn;
        double radial = 1 + k1 * r2 + k2 * r2 * r2;

        double dx = 2 * p1 * xn * yn + p2 * (r2 + 2 * xn * xn);
        double dy = p1 * (r2 + 2 * yn * yn) + 2 * p2 * xn * yn;

        xn = (norm.first - dx) / radial;
        yn = (norm.second - dy) / radial;
    }

    return normalizedToPixel(xn, yn);
}

// Block类XML读写实现
bool Block::loadFromXML(const std::string& filename) {
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(filename.c_str()) != tinyxml2::XML_SUCCESS) {
        std::cout << "XML文件加载失败: " << filename << std::endl;
        std::cout << "错误信息: " << doc.ErrorStr() << std::endl;
        return false;
    }

    tinyxml2::XMLElement* root = doc.RootElement();
    if (!root) return false;

    tinyxml2::XMLElement* blockElem = root->FirstChildElement("Block");
    if (!blockElem) return false;

    // 读取相机参数
    tinyxml2::XMLElement* photogroups = blockElem->FirstChildElement("Photogroups");
    if (photogroups) {
        tinyxml2::XMLElement* photogroup = photogroups->FirstChildElement("Photogroup");
        if (photogroup) {
            // 读取内参
            tinyxml2::XMLElement* focalLength = photogroup->FirstChildElement("FocalLength");
            if (focalLength) {
                double f = focalLength->DoubleText();
                camera.intrinsic.data[0][0] = f;
                camera.intrinsic.data[1][1] = f;
            }

            // 读取主点
            tinyxml2::XMLElement* principalPoint = photogroup->FirstChildElement("PrincipalPoint");
            if (principalPoint) {
                tinyxml2::XMLElement* x = principalPoint->FirstChildElement("x");
                tinyxml2::XMLElement* y = principalPoint->FirstChildElement("y");
                if (x) camera.intrinsic.data[0][2] = x->DoubleText();
                if (y) camera.intrinsic.data[1][2] = y->DoubleText();
            }

            // 读取畸变系数
            tinyxml2::XMLElement* distortion = photogroup->FirstChildElement("Distortion");
            if (distortion) {
                tinyxml2::XMLElement* k1Elem = distortion->FirstChildElement("K1");
                tinyxml2::XMLElement* k2Elem = distortion->FirstChildElement("K2");
                tinyxml2::XMLElement* p1Elem = distortion->FirstChildElement("P1");
                tinyxml2::XMLElement* p2Elem = distortion->FirstChildElement("P2");

                if (k1Elem) camera.k1 = k1Elem->DoubleText();
                if (k2Elem) camera.k2 = k2Elem->DoubleText();
                if (p1Elem) camera.p1 = p1Elem->DoubleText();
                if (p2Elem) camera.p2 = p2Elem->DoubleText();
            }
        }
    }

    // 读取特征点
    tinyxml2::XMLElement* tiePointsElem = blockElem->FirstChildElement("TiePoints");
    if (tiePointsElem) {
        for (tinyxml2::XMLElement* tiePoint = tiePointsElem->FirstChildElement("TiePoint");
            tiePoint;
            tiePoint = tiePoint->NextSiblingElement("TiePoint")) {

            TiePnt tp;

            // 读取位置
            tinyxml2::XMLElement* position = tiePoint->FirstChildElement("Position");
            if (position) {
                tinyxml2::XMLElement* x = position->FirstChildElement("x");
                tinyxml2::XMLElement* y = position->FirstChildElement("y");
                tinyxml2::XMLElement* z = position->FirstChildElement("z");

                if (x && y && z) {
                    tp.position = Vec_t(
                        x->DoubleText(),
                        y->DoubleText(),
                        z->DoubleText()
                    );
                }
            }

            // 读取颜色
            tinyxml2::XMLElement* color = tiePoint->FirstChildElement("Color");
            if (color) {
                tinyxml2::XMLElement* r = color->FirstChildElement("Red");
                tinyxml2::XMLElement* g = color->FirstChildElement("Green");
                tinyxml2::XMLElement* b = color->FirstChildElement("Blue");

                if (r && g && b) {
                    tp.color = Color(
                        r->DoubleText(),
                        g->DoubleText(),
                        b->DoubleText()
                    );
                }
            }

            // 读取观测点
            for (tinyxml2::XMLElement* measurement = tiePoint->FirstChildElement("Measurement");
                measurement;
                measurement = measurement->NextSiblingElement("Measurement")) {

                tinyxml2::XMLElement* photoId = measurement->FirstChildElement("PhotoId");
                tinyxml2::XMLElement* x = measurement->FirstChildElement("x");
                tinyxml2::XMLElement* y = measurement->FirstChildElement("y");

                if (photoId && x && y) {
                    tp.observations.push_back(Obser(
                        photoId->IntText(),
                        x->DoubleText(),
                        y->DoubleText()
                    ));
                }
            }

            tiepoints.push_back(tp);
        }
    }

    return true;
}

bool Block::saveToXML(const std::string& filename) const {
    tinyxml2::XMLDocument doc;

    // 创建根元素
    tinyxml2::XMLElement* root = doc.NewElement("ContextCapture");
    doc.InsertFirstChild(root);

    // 创建Block元素
    tinyxml2::XMLElement* blockElem = doc.NewElement("Block");
    root->InsertEndChild(blockElem);

    // 保存相机参数
    tinyxml2::XMLElement* photogroups = doc.NewElement("Photogroups");
    blockElem->InsertEndChild(photogroups);

    tinyxml2::XMLElement* photogroup = doc.NewElement("Photogroup");
    photogroups->InsertEndChild(photogroup);

    // 保存内参
    tinyxml2::XMLElement* focalLength = doc.NewElement("FocalLength");
    focalLength->SetText(camera.intrinsic.data[0][0]);
    photogroup->InsertEndChild(focalLength);

    // ... 保存其他参数 ...

    return doc.SaveFile(filename.c_str()) == tinyxml2::XML_SUCCESS;
}

Eigen::Matrix3d Camera::calculateDLT(const std::vector<std::pair<Point3D, std::pair<double, double>>>& correspondences) const {
    // 1. 数据归一化
    Eigen::Vector3d mean_3d = Eigen::Vector3d::Zero();
    Eigen::Vector2d mean_2d = Eigen::Vector2d::Zero();

    // 计算质心
    for (const auto& corr : correspondences) {
        mean_3d += Eigen::Vector3d(corr.first.x, corr.first.y, corr.first.z);
        mean_2d += Eigen::Vector2d(corr.second.first, corr.second.second);
    }
    mean_3d /= correspondences.size();
    mean_2d /= correspondences.size();

    // 计算缩放因子
    double scale_3d = 0, scale_2d = 0;
    for (const auto& corr : correspondences) {
        scale_3d += ((Eigen::Vector3d(corr.first.x, corr.first.y, corr.first.z) - mean_3d).norm());
        scale_2d += ((Eigen::Vector2d(corr.second.first, corr.second.second) - mean_2d).norm());
    }
    scale_3d = sqrt(2.0) * correspondences.size() / scale_3d;
    scale_2d = sqrt(2.0) * correspondences.size() / scale_2d;

    // 2. 构建A矩阵
    Eigen::MatrixXd A(correspondences.size() * 2, 9);

    for (size_t i = 0; i < correspondences.size(); i++) {
        // 归一化坐标
        Eigen::Vector3d X((correspondences[i].first.x - mean_3d(0)) * scale_3d,
            (correspondences[i].first.y - mean_3d(1)) * scale_3d,
            1.0);
        Eigen::Vector2d x((correspondences[i].second.first - mean_2d(0)) * scale_2d,
            (correspondences[i].second.second - mean_2d(1)) * scale_2d);

        // 填充A矩阵
        A.block<1, 3>(i * 2, 0) = X.transpose();
        A.block<1, 3>(i * 2, 3) = Eigen::Vector3d::Zero().transpose();
        A.block<1, 3>(i * 2, 6) = -x(0) * X.transpose();

        A.block<1, 3>(i * 2 + 1, 0) = Eigen::Vector3d::Zero().transpose();
        A.block<1, 3>(i * 2 + 1, 3) = X.transpose();
        A.block<1, 3>(i * 2 + 1, 6) = -x(1) * X.transpose();
    }

    // 3. SVD求解
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
    Eigen::VectorXd h = svd.matrixV().col(8);

    // 4. 重构H矩阵
    Eigen::Matrix3d H;
    H << h(0), h(1), h(2),
        h(3), h(4), h(5),
        h(6), h(7), h(8);

    // 5. 反归一化
    Eigen::Matrix3d T_2d = Eigen::Matrix3d::Identity();
    T_2d.block<2, 2>(0, 0) *= scale_2d;
    T_2d.block<2, 1>(0, 2) = -scale_2d * mean_2d;

    Eigen::Matrix3d T_3d = Eigen::Matrix3d::Identity();
    T_3d.block<2, 2>(0, 0) *= scale_3d;
    T_3d.block<2, 1>(0, 2) = -scale_3d * mean_3d.head<2>();

    H = T_2d.inverse() * H * T_3d;

    // 归一化H矩阵
    H /= H(2, 2);

    return H;
}

std::pair<double, double> Camera::projectPointWithDLT(const Point3D& point, const Eigen::Matrix3d& H) const {
    // 将3D点转换为齐次坐标
    Eigen::Vector3d X(point.x, point.y, 1.0);

    // 应用投影变换
    Eigen::Vector3d x = H * X;

    // 齐次坐标归一化
    if (abs(x(2)) > 1e-10) {
        double px = x(0) / x(2);
        double py = x(1) / x(2);
        return std::make_pair(px, py);
    }

    return std::make_pair(-1.0, -1.0);
}

std::vector<Eigen::Matrix4d> Camera::solveP3P(const std::vector<Point3D>& points3D,
    const std::vector<std::pair<double, double>>& points2D) const {
    std::vector<Eigen::Matrix4d> solutions;

    if (points3D.size() < 3 || points2D.size() < 3) {
        return solutions;
    }

    // 将2D点转换为归一化坐标
    std::vector<Eigen::Vector3d> rays;
    for (const auto& pt : points2D) {
        auto norm = pixelToNormalized(pt.first, pt.second);
        Eigen::Vector3d ray(norm.first, norm.second, 1.0);
        ray.normalize();
        rays.push_back(ray);
    }

    // 计算三个向量之间的夹角余弦
    double cos12 = rays[0].dot(rays[1]);
    double cos13 = rays[0].dot(rays[2]);
    double cos23 = rays[1].dot(rays[2]);

    // 计算三个3D点之间的距离
    double d12 = (Eigen::Vector3d(points3D[0].x, points3D[0].y, points3D[0].z) -
        Eigen::Vector3d(points3D[1].x, points3D[1].y, points3D[1].z)).norm();
    double d13 = (Eigen::Vector3d(points3D[0].x, points3D[0].y, points3D[0].z) -
        Eigen::Vector3d(points3D[2].x, points3D[2].y, points3D[2].z)).norm();
    double d23 = (Eigen::Vector3d(points3D[1].x, points3D[1].y, points3D[1].z) -
        Eigen::Vector3d(points3D[2].x, points3D[2].y, points3D[2].z)).norm();

    // 构建并求解四次方程
    Eigen::Matrix4d A;
    // ... 构建系数矩阵A

    // 求解四次方程得到可能的解
    Eigen::EigenSolver<Eigen::Matrix4d> solver(A);

    // 对每个实根构建位姿矩阵
    for (int i = 0; i < 4; i++) {
        if (solver.eigenvalues()(i).imag() == 0) {
            Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
            // ... 从特征值构位姿矩阵
            solutions.push_back(pose);
        }
    }

    return solutions;
}

Eigen::Matrix4d Camera::selectBestP3PSolution(const std::vector<Eigen::Matrix4d>& solutions,
    const std::vector<Point3D>& points3D,
    const std::vector<std::pair<double, double>>& points2D) const {
    double min_error = std::numeric_limits<double>::max();
    Eigen::Matrix4d best_solution = Eigen::Matrix4d::Identity();

    for (const auto& solution : solutions) {
        double error = calculateReprojectionError(solution, points3D, points2D);
        if (error < min_error) {
            min_error = error;
            best_solution = solution;
        }
    }

    return best_solution;
}

double Camera::calculateReprojectionError(const Eigen::Matrix4d& pose,
    const std::vector<Point3D>& points3D,
    const std::vector<std::pair<double, double>>& points2D) const {
    double total_error = 0.0;

    for (size_t i = 0; i < points3D.size(); i++) {
        // 将3D点转换到相机坐标系
        Eigen::Vector4d pt3d(points3D[i].x, points3D[i].y, points3D[i].z, 1.0);
        Eigen::Vector4d pt_cam = pose * pt3d;

        // 投影到图像平面
        if (pt_cam(2) > 0) {
            double x = pt_cam(0) / pt_cam(2);
            double y = pt_cam(1) / pt_cam(2);

            // 计算重投影误差
            double dx = x - points2D[i].first;
            double dy = y - points2D[i].second;
            total_error += sqrt(dx * dx + dy * dy);
        }
    }

    return total_error / points3D.size();
}

// 计算两张影像之间的刚体变换参数
RigidTransform Camera::calculateRigidTransform(const Photo& photo1, const Photo& photo2) const {
    RigidTransform transform;

    // 将Mat_R转换为Eigen::Matrix3d
    Eigen::Matrix3d R1, R2;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R1(i, j) = photo1.rotation.data[i][j];
            R2(i, j) = photo2.rotation.data[i][j];
        }
    }

    // 将Vec_t转换为Eigen::Vector3d
    Eigen::Vector3d t1(photo1.position.x, photo1.position.y, photo1.position.z);
    Eigen::Vector3d t2(photo2.position.x, photo2.position.y, photo2.position.z);

    // 算相对旋转矩阵：R12 = R2 * R1.transpose()
    transform.R = R2 * R1.transpose();

    // 计算相对平移向量：t12 = t2 - R12 * t1
    transform.t = t2 - transform.R * t1;

    return transform;
}

// 使用刚体变换参数计算另一张影像的位姿
Photo Camera::calculateNewPose(const Photo& photo1, const RigidTransform& transform) const {
    Photo newPhoto;

    // 将Mat_R转换为Eigen::Matrix3d
    Eigen::Matrix3d R1;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R1(i, j) = photo1.rotation.data[i][j];
        }
    }

    // 将Vec_t转换为Eigen::Vector3d
    Eigen::Vector3d t1(photo1.position.x, photo1.position.y, photo1.position.z);

    // 计算新的旋转矩阵：R2 = R12 * R1
    Eigen::Matrix3d R2 = transform.R * R1;

    // 计算新的平移向量：t2 = R12 * t1 + t12
    Eigen::Vector3d t2 = transform.R * t1 + transform.t;

    // 将结果存回Photo结构
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            newPhoto.rotation.data[i][j] = R2(i, j);
        }
        newPhoto.translation.data[i][0] = t2(i);
    }

    newPhoto.position = Vec_t(t2.x(), t2.y(), t2.z());
    newPhoto.euler_angles = rotationMatrixToEulerAngles(
        R2(0, 0), R2(0, 1), R2(0, 2),
        R2(1, 0), R2(1, 1), R2(1, 2),
        R2(2, 0), R2(2, 1), R2(2, 2)
    );

    return newPhoto;
}

// 实现欧拉角计算函数
Vec_t rotationMatrixToEulerAngles(double r00, double r01, double r02,
    double r10, double r11, double r12,
    double r20, double r21, double r22) {
    Vec_t euler;

    // 计算欧角 (按照 ZYX 序)
    euler.y = asin(r02);  // pitch
    euler.x = atan2(-r12, r22);  // roll
    euler.z = atan2(-r01, r00);  // yaw

    return euler;
}

EssentialMatrixResult Camera::calculateEssentialMatrix(
    const Photo& photo1,
    const Photo& photo2,
    const std::vector<TiePnt>& tiepoints,
    const Mat_K& K) const {

    EssentialMatrixResult result;

    // 构建特征点对应关系
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> matches;
    for (const auto& tp : tiepoints) {
        Eigen::Vector2d pt1, pt2;
        bool found1 = false, found2 = false;

        for (const auto& obs : tp.observations) {
            if (obs.photoId == 0) {
                pt1 = Eigen::Vector2d(obs.x, obs.y);
                found1 = true;
            }
            if (obs.photoId == 1) {
                pt2 = Eigen::Vector2d(obs.x, obs.y);
                found2 = true;
            }
        }

        if (found1 && found2) {
            matches.push_back({ pt1, pt2 });
        }
    }

    // 转换K矩阵为Eigen格式
    Eigen::Matrix3d K_eigen;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            K_eigen(i, j) = K.data[i][j];
        }
    }

    // 计算本质矩阵
    result.E = EssentialMatrixSolver::calculateEssentialMatrix(matches, K_eigen);

    // 从本质矩阵恢复R和t
    std::vector<Eigen::Matrix3d> Rs;
    std::vector<Eigen::Vector3d> ts;
    EssentialMatrixSolver::decomposeEssentialMatrix(result.E, Rs, ts);

    // 选择最佳的R和t
    EssentialMatrixSolver::selectBestRT(Rs, ts, matches, K, result.R, result.t);

    return result;
}
} // namespace slam