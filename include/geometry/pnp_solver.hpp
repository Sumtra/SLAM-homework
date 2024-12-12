#ifndef PNP_SOLVER_HPP
#define PNP_SOLVER_HPP

#include "../types/common_types.hpp"
#include <vector>
#include <Eigen/Dense>
#include "../geometry/camera.hpp"
#include "../types/slam_types.hpp"

namespace slam {

struct Point2D {
    double x;
    double y;
    Point2D(double x_ = 0, double y_ = 0) : x(x_), y(y_) {}
};

class PnPSolver {
public:
    // DLT算法求解位姿
    static bool solveDLT(
        const std::vector<Point3D>& points3D,
        const std::vector<Obser>& observations,
        const Mat_K& K,
        Mat_R& R,
        Vec_t& t,
        bool silent = false
    );

    // P3P算法求解位姿
    static bool solveP3P(
        const std::vector<Point3D>& points3D,
        const std::vector<Obser>& observations,
        const Mat_K& K,
        Mat_R& R,
        Vec_t& t
    );

    // RANSAC+DLT
    static bool ransacDLT(
        const std::vector<Point3D>& points3D,
        const std::vector<Obser>& observations,
        const Mat_K& K,
        Mat_R& R,
        Vec_t& t
    );

    // RANSAC+P3P
    static bool ransacP3P(
        const std::vector<Point3D>& points3D,
        const std::vector<Obser>& observations,
        const Mat_K& K,
        Mat_R& R,
        Vec_t& t
    );

    // 计算重投影误差
    static double computeReprojectionError(
        const std::vector<Point3D>& points3D,
        const std::vector<Obser>& observations,
        const Mat_K& K,
        const Mat_R& R,
        const Vec_t& t
    );

    // 将函数声明为静态
    static void convertIntrinsicMatrix(Mat_K& K);

    // 添加非线性优化函数声明
    static void refinePose(
        const std::vector<Point3D>& points3D,
        const std::vector<Obser>& observations,
        const Mat_K& K,
        Mat_R& R,
        Vec_t& t
    );

    // 调试函数
    static void debugCoordinateSystem(const Mat_R& R, const Vec_t& t);
    static void printK(const Mat_K& K);

private:
    static Eigen::Vector3d convertT(const Vec_t& t);
    static Mat_R convertToMatR(const Eigen::Matrix3d& R);
    static Vec_t convertToVecT(const Eigen::Vector3d& t);

    // 检查点是否在相机前方
   static bool checkPointInFrontOfCamera(
       const Point3D& point,
       const Mat_R& R,
       const Vec_t& t
   );

    // 计算单个点的重投影误差
    static double computeReprojectionErrorForPoint(
        const Point3D& point3D,
        const Obser& observation,
        const Mat_K& K,
        const Mat_R& R,
        const Vec_t& t,
        bool silent = false
    );

    // 归一化点
    static void normalizePoints(
        const std::vector<Point3D>& points3D,
        std::vector<Point3D>& normalized_points3D,
        double& scale3D,
        Eigen::Vector3d& centroid3D
    );

    // 计算质心
    static Point3D computeCentroid(const std::vector<Point3D>& points);
    static Obser computeCentroid(const std::vector<Obser>& observations);

    static double computeReprojectionErrorSilent(
        const std::vector<Point3D>& points3D,
        const std::vector<Obser>& observations,
        const Mat_K& K,
        const Mat_R& R,
        const Vec_t& t);

    static Point2D project(
        const Point3D& point3D,
        const Mat_K& K,
        const Mat_R& R,
        const Vec_t& t);

    static bool checkauto_Rotation(const Eigen::Matrix3d& R);

    // 添加目标z值作为静态常量
    static constexpr double target_z = 581.487;  // 根据输入数据的第一个点的z值设置
};

} // namespace slam

#endif // PNP_SOLVER_HPP 