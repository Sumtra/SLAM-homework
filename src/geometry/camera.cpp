#include "../../include/geometry/essential_matrix.hpp"
#include "../../include/geometry/camera.hpp"
#include "../../include/types/slam_types.hpp"
#include <cmath>
#include <gdal_priv.h>
#include <gdalwarper.h>

namespace slam {

// 添加新的成员函数：使用GDAL读取和去畸变影像
GDALDataset* Camera::undistortImage(const std::string& inputPath, const std::string& outputPath) const {
    // 初始化GDAL
    GDALAllRegister();
    
    // 打开输入影像
    GDALDataset* inputDS = (GDALDataset*)GDALOpen(inputPath.c_str(), GA_ReadOnly);
    if (!inputDS) {
        throw std::runtime_error("无法打开输入影像");
    }

    // 获取影像信息
    int width = inputDS->GetRasterXSize();
    int height = inputDS->GetRasterYSize();
    int bands = inputDS->GetRasterCount();

    // 创建输出影像
    GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("GTiff");
    GDALDataset* outputDS = driver->Create(outputPath.c_str(), 
                                         width, height, bands, 
                                         GDT_Byte, nullptr);
    
    if (!outputDS) {
        GDALClose(inputDS);
        throw std::runtime_error("无法创建输出影像");
    }

    // 分块处理影像
    const int blockSize = 1024; // 可以根据需要调整块大小
    for (int y = 0; y < height; y += blockSize) {
        for (int x = 0; x < width; x += blockSize) {
            int currentBlockWidth = std::min(blockSize, width - x);
            int currentBlockHeight = std::min(blockSize, height - y);
            
            processImageBlock(inputDS, outputDS, x, y, 
                            currentBlockWidth, currentBlockHeight);
        }
    }

    // 复制投影信息
    outputDS->SetProjection(inputDS->GetProjectionRef());

    // 正确使用 GetGeoTransform
    double geoTransform[6];  // 用于存储地理变换参数
    if (inputDS->GetGeoTransform(geoTransform) == CE_None) {
        outputDS->SetGeoTransform(geoTransform);
    }

    // 关闭输入影像
    GDALClose(inputDS);
    
    return outputDS;
}

void Camera::processImageBlock(GDALDataset* input, GDALDataset* output,
                             int startX, int startY, 
                             int blockWidth, int blockHeight) const {
    int bands = input->GetRasterCount();
    std::vector<float> buffer(blockWidth * blockHeight);
    std::vector<float> outBuffer(blockWidth * blockHeight);

    // 对每个波段进行处理
    for (int band = 1; band <= bands; ++band) {
        GDALRasterBand* inBand = input->GetRasterBand(band);
        GDALRasterBand* outBand = output->GetRasterBand(band);

        // 读取数据块
        inBand->RasterIO(GF_Read, startX, startY, 
                        blockWidth, blockHeight,
                        buffer.data(), blockWidth, blockHeight, 
                        GDT_Float32, 0, 0);

        // 对块内每个像素进行去畸变
        for (int y = 0; y < blockHeight; ++y) {
            for (int x = 0; x < blockWidth; ++x) {
                // 计算原始影像坐标
                double origX = startX + x;
                double origY = startY + y;

                // 应用去畸变
                auto [undistX, undistY] = undistortPoint(origX, origY);

                // 检查是否在影像范围内
                if (undistX >= 0 && undistX < input->GetRasterXSize() &&
                    undistY >= 0 && undistY < input->GetRasterYSize()) {
                    // 使用双线性插值获取像素值
                    int x0 = static_cast<int>(undistX);
                    int y0 = static_cast<int>(undistY);
                    double dx = undistX - x0;
                    double dy = undistY - y0;

                    // 获取周围四个像素的值并进行插值
                    float val00, val01, val10, val11;
                    inBand->RasterIO(GF_Read, x0, y0, 1, 1, &val00, 1, 1, GDT_Float32, 0, 0);
                    inBand->RasterIO(GF_Read, x0+1, y0, 1, 1, &val10, 1, 1, GDT_Float32, 0, 0);
                    inBand->RasterIO(GF_Read, x0, y0+1, 1, 1, &val01, 1, 1, GDT_Float32, 0, 0);
                    inBand->RasterIO(GF_Read, x0+1, y0+1, 1, 1, &val11, 1, 1, GDT_Float32, 0, 0);

                    float value = (1-dx)*(1-dy)*val00 + dx*(1-dy)*val10 + 
                                (1-dx)*dy*val01 + dx*dy*val11;
                    
                    outBuffer[y * blockWidth + x] = value;
                } else {
                    outBuffer[y * blockWidth + x] = 0; // 超出范围的像素设为0
                }
            }
        }

        // 写入处理后的数据块
        outBand->RasterIO(GF_Write, startX, startY, 
                         blockWidth, blockHeight,
                         outBuffer.data(), blockWidth, blockHeight, 
                         GDT_Float32, 0, 0);
    }
}

// 坐标转换函数
std::pair<double, double> Camera::pixelToNormalized(double x, double y) const {
    // 像素坐标转归一化坐标
    double xn = (x - intrinsic.data[0][2]) / intrinsic.data[0][0];
    double yn = (y - intrinsic.data[1][2]) / intrinsic.data[1][1];
    return {xn, yn};
}

std::pair<double, double> Camera::normalizedToPixel(double xn, double yn) const {
    // 归一化坐标转像素坐标
    double x = xn * intrinsic.data[0][0] + intrinsic.data[0][2];
    double y = yn * intrinsic.data[1][1] + intrinsic.data[1][2];
    return {x, y};
}

// 畸变处理函数
std::pair<double, double> Camera::undistortPoint(double x, double y) const {
    // 先转换为归一化坐标
    auto [xn, yn] = pixelToNormalized(x, y);
    
    // 迭代去畸变
    double x_distorted = xn;
    double y_distorted = yn;
    
    for (int i = 0; i < 10; i++) {
        double r2 = x_distorted * x_distorted + y_distorted * y_distorted;
        double radial = 1 + k1 * r2 + k2 * r2 * r2;
        double tangential_x = 2 * p1 * x_distorted * y_distorted + p2 * (r2 + 2 * x_distorted * x_distorted);
        double tangential_y = p1 * (r2 + 2 * y_distorted * y_distorted) + 2 * p2 * x_distorted * y_distorted;
        
        x_distorted = (xn - tangential_x) / radial;
        y_distorted = (yn - tangential_y) / radial;
    }
    
    // 转回像素坐标
    return normalizedToPixel(x_distorted, y_distorted);
}

std::pair<double, double> Camera::distortPoint(double x, double y) const {
    // 先转换为归一化坐标
    auto [xn, yn] = pixelToNormalized(x, y);
    
    // 添加畸变
    double r2 = xn * xn + yn * yn;
    double radial = 1 + k1 * r2 + k2 * r2 * r2;
    double tangential_x = 2 * p1 * xn * yn + p2 * (r2 + 2 * xn * xn);
    double tangential_y = p1 * (r2 + 2 * yn * yn) + 2 * p2 * xn * yn;
    
    double x_distorted = xn * radial + tangential_x;
    double y_distorted = yn * radial + tangential_y;
    
    // 转回像素坐标
    return normalizedToPixel(x_distorted, y_distorted);
}

// DLT相关函数
Eigen::Matrix3d Camera::calculateDLT(
    const std::vector<std::pair<Point3D, std::pair<double, double>>>& correspondences) const {
    if (correspondences.size() < 6) {
        throw std::runtime_error("Not enough points");
    }

    // 添加数据归一化步骤
    // 1. 计算3D点的质心和尺度
    Eigen::Vector3d centroid3D = Eigen::Vector3d::Zero();
    double scale3D = 0;
    
    for (const auto& corr : correspondences) {
        centroid3D += Eigen::Vector3d(corr.first.x, corr.first.y, corr.first.z);
    }
    centroid3D /= correspondences.size();
    
    // 2. 构建归一化矩阵
    // 3. 对点进行归一化
    // 4. 使用归一化后的点计算H
    // 5. 反归一化得到最终结果

    // 构建A矩阵
    Eigen::MatrixXd A(correspondences.size() * 2, 9);
    
    for (size_t i = 0; i < correspondences.size(); i++) {
        const auto& p3d = correspondences[i].first;
        const auto& p2d = correspondences[i].second;
        
        // 归一化坐标
        double x = (p2d.first - intrinsic.data[0][2]) / intrinsic.data[0][0];
        double y = (p2d.second - intrinsic.data[1][2]) / intrinsic.data[1][1];
        double X = p3d.x;
        double Y = p3d.y;
        double Z = p3d.z;

        // 填充A矩阵
        A.row(i * 2) << X, Y, Z, 0, 0, 0, -x*X, -x*Y, -x*Z;
        A.row(i * 2 + 1) << 0, 0, 0, X, Y, Z, -y*X, -y*Y, -y*Z;
    }

    // SVD求解
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
    Eigen::VectorXd h = svd.matrixV().col(8);

    // 构建并返回变换矩阵
    Eigen::Matrix3d H;
    H << h(0), h(1), h(2),
         h(3), h(4), h(5),
         h(6), h(7), h(8);
    
    return H / H(2,2);
}

std::pair<double, double> Camera::projectPointWithDLT(
    const Point3D& point, const Eigen::Matrix3d& H) const {
    // 构建齐次坐标
    Eigen::Vector3d p(point.x, point.y, point.z);
    
    // 应用变换
    Eigen::Vector3d projected = H * p;
    
    // 归一化
    if (std::abs(projected(2)) > 1e-10) {
        return {projected(0)/projected(2), projected(1)/projected(2)};
    }
    
    throw std::runtime_error("Invalid projection");
}

// P3P相关函数
std::vector<Eigen::Matrix4d> Camera::solveP3P(
    const std::vector<Point3D>& points3D,
    const std::vector<std::pair<double, double>>& points2D) const {
    
    if (points3D.size() != 3 || points2D.size() != 3) {
        throw std::runtime_error("P3P requires exactly 3 points");
    }

    // 将2D点转换为归一化相机坐标
    std::vector<Eigen::Vector3d> normalized_points;
    for (const auto& p : points2D) {
        double x = (p.first - intrinsic.data[0][2]) / intrinsic.data[0][0];
        double y = (p.second - intrinsic.data[1][2]) / intrinsic.data[1][1];
        normalized_points.push_back(Eigen::Vector3d(x, y, 1).normalized());
    }

    // TODO: 实现P3P的核心算法
    // 1. 计算三个控制点之间的距离
    // 2. 建立并求解四次方程
    // 3. 计算可能的R和t

    return std::vector<Eigen::Matrix4d>();  // 临时返回空结果
}

Eigen::Matrix4d Camera::selectBestP3PSolution(
    const std::vector<Eigen::Matrix4d>& solutions,
    const std::vector<Point3D>& points3D,
    const std::vector<std::pair<double, double>>& points2D) const {
    // ... 已经在 pnp_solver.cpp 中实现
    return Eigen::Matrix4d::Identity();  // 临时返回值
}

double Camera::calculateReprojectionError(
    const Eigen::Matrix4d& pose,
    const std::vector<Point3D>& points3D,
    const std::vector<std::pair<double, double>>& points2D) const {
    // ... 已经在 pnp_solver.cpp 中实现
    return 0.0;  // 临时返回值
}

// 刚体变换相关函数
RigidTransform Camera::calculateRigidTransform(
    const Photo& photo1, const Photo& photo2) const {
    // ... 已经在 rigid_transform.cpp 中实现
    return RigidTransform();  // 临时返回值
}

Photo Camera::calculateNewPose(
    const Photo& photo1, const RigidTransform& transform) const {
    // ... 已经在 rigid_transform.cpp 中实现
    return Photo();  // 临时返回值
}

// 本质矩阵相关函数
EssentialMatrixResult Camera::calculateEssentialMatrix(
    const Photo& photo1,
    const Photo& photo2,
    const std::vector<TiePnt>& tiepoints) const
{
    // 使用当前相机的内参矩阵
    return calculateEssentialMatrixWithK(photo1, photo2, tiepoints, this->intrinsic);
}

EssentialMatrixResult Camera::calculateEssentialMatrixWithK(
    const Photo& photo1,
    const Photo& photo2,
    const std::vector<TiePnt>& tiepoints,
    const Mat_K& K) const 
{
    EssentialMatrixResult result;

    // 获取匹配点对
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

    
    Eigen::Matrix3d K_eigen = K;

    // 计算本质矩阵
    result.E = EssentialMatrixSolver::calculateEssentialMatrix(matches, K);

    // 分解本质矩阵得到可能的R和t
    std::vector<Eigen::Matrix3d> Rs;
    std::vector<Eigen::Vector3d> ts;
    EssentialMatrixSolver::decomposeEssentialMatrix(result.E, Rs, ts);

    // 选择最佳的R和t
    EssentialMatrixSolver::selectBestRT(Rs, ts, matches, K_eigen, result.R, result.t);

    return result;
}
/*
// 3. RANSAC + DLT
Eigen::Matrix3d Camera::ransacDLT(
    const std::vector<std::pair<Point3D, std::pair<double, double>>>& correspondences,
    std::vector<bool>& inliers,
    double threshold = 0.01,
    int max_iterations = 1000) const {
    
    if (correspondences.size() < 6) {
        throw std::runtime_error("Not enough points");
    }

    std::mt19937 gen(std::random_device{}());
    std::uniform_int_distribution<> dis(0, correspondences.size() - 1);

    Eigen::Matrix3d best_H = Eigen::Matrix3d::Identity();
    int best_inlier_count = 0;
    inliers.assign(correspondences.size(), false);

    for (int iter = 0; iter < max_iterations; ++iter) {
        // 随机选择6个点
        std::vector<std::pair<Point3D, std::pair<double, double>>> sample;
        for (int i = 0; i < 6; ++i) {
            sample.push_back(correspondences[dis(gen)]);
        }

        // 计算DLT
        Eigen::Matrix3d H = calculateDLT(sample);

        // 统计内点
        int inlier_count = 0;
        std::vector<bool> current_inliers(correspondences.size(), false);
        
        for (size_t i = 0; i < correspondences.size(); ++i) {
            const auto& corr = correspondences[i];
            auto projected = projectPointWithDLT(corr.first, H);
            
            double error = std::hypot(
                projected.first - corr.second.first,
                projected.second - corr.second.second
            );

            if (error < threshold) {
                current_inliers[i] = true;
                inlier_count++;
            }
        }

        // 更新最佳结果
        if (inlier_count > best_inlier_count) {
            best_inlier_count = inlier_count;
            best_H = H;
            inliers = current_inliers;
        }
    }

    return best_H;
}
*/
/*
// 4. RANSAC + P3P
Eigen::Matrix4d Camera::ransacP3P(
    const std::vector<Point3D>& points3D,
    const std::vector<std::pair<double, double>>& points2D,
    std::vector<bool>& inliers,
    double threshold = 0.01,
    int max_iterations = 1000) const {
    
    if (points3D.size() < 3 || points2D.size() < 3) {
        throw std::runtime_error("Not enough points");
    }

    std::mt19937 gen(std::random_device{}());
    std::uniform_int_distribution<> dis(0, points3D.size() - 1);

    Eigen::Matrix4d best_pose = Eigen::Matrix4d::Identity();
    int best_inlier_count = 0;
    inliers.assign(points3D.size(), false);

    // TODO: 实现RANSAC+P3P
    // 1. 随机选择3个点
    // 2. 使用P3P求解可能的位姿
    // 3. 验证所有点，统计内点
    // 4. 更新最佳结果

    return best_pose;
}
*/
} // namespace slam; 