#ifndef SLAM_CAMERA_HPP
#define SLAM_CAMERA_HPP

#include <vector>
#include <utility>
#include <Eigen/Dense>
#include "../types/slam_types.hpp"
#include "../types/common_types.hpp"
#include "essential_matrix.hpp"  
#include <gdal_priv.h>
//#include <opencv2/opencv.hpp>
namespace slam {

class Camera {
public:
    // 成员变量
    Mat_K intrinsic;
    double k1 = 0.0;
    double k2 = 0.0;
    double p1 = 0.0;
    double p2 = 0.0;
    std::vector<Photo> photos;

    // 成员函数声明
    std::pair<double, double> pixelToNormalized(double x, double y) const;
    std::pair<double, double> normalizedToPixel(double xn, double yn) const;
    std::pair<double, double> undistortPoint(double x, double y) const;
    std::pair<double, double> distortPoint(double x, double y) const;
    
    Eigen::Matrix3d calculateDLT(
        const std::vector<std::pair<Point3D, std::pair<double, double>>>& correspondences) const;
    std::pair<double, double> projectPointWithDLT(
        const Point3D& point, const Eigen::Matrix3d& H) const;
    
    std::vector<Eigen::Matrix4d> solveP3P(
        const std::vector<Point3D>& points3D,
        const std::vector<std::pair<double, double>>& points2D) const;
    
    Eigen::Matrix4d selectBestP3PSolution(
        const std::vector<Eigen::Matrix4d>& solutions,
        const std::vector<Point3D>& points3D,
        const std::vector<std::pair<double, double>>& points2D) const;
    
    double calculateReprojectionError(
        const Eigen::Matrix4d& pose,
        const std::vector<Point3D>& points3D,
        const std::vector<std::pair<double, double>>& points2D) const;
    
    RigidTransform calculateRigidTransform(
        const Photo& photo1, const Photo& photo2) const;
    
    Photo calculateNewPose(
        const Photo& photo1, const RigidTransform& transform) const;

    // 本质矩阵计算函数
    EssentialMatrixResult calculateEssentialMatrix(
        const Photo& photo1,
        const Photo& photo2,
        const std::vector<TiePnt>& tiepoints) const;
    
    EssentialMatrixResult calculateEssentialMatrixWithK(
        const Photo& photo1, 
        const Photo& photo2, 
        const std::vector<TiePnt>& tiepoints, 
        const Mat_K& K) const;
    
    // 使用GDAL
    GDALDataset* undistortImage(const std::string& inputPath, const std::string& outputPath) const;


private:
// 辅助函数
    void processImageBlock(GDALDataset* input, GDALDataset* output, 
                         int startX, int startY, int blockWidth, int blockHeight) const;

    // 可能需要的辅助函数
    double calculateReprojectionError(
        const Eigen::Matrix4d& pose,
        const Point3D& point3D,
        const std::pair<double, double>& point2D) const;
};



} // namespace slam

#endif // SLAM_CAMERA_HPP
