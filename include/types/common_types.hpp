#ifndef COMMON_TYPES_HPP
#define COMMON_TYPES_HPP

#include <Eigen/Dense>
#include <vector>
#include <string>

namespace slam {

// 基础几何结构
struct Point3D {
    double x, y, z;
    Point3D(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
};

struct Color {
    double r, g, b;
    Color(double r = 0, double g = 0, double b = 0) : r(r), g(g), b(b) {}
};

struct Vec_t {
    double x, y, z;
    Vec_t(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}

    Vec_t& operator=(const Point3D& p) {
        x = p.x;
        y = p.y;
        z = p.z;
        return *this;
    }
};

// 矩阵结构
// 矩阵结构
struct Mat_K {
    double data[3][3];

    // 构造函数声明
    Mat_K();

    // 类型转换运算符声明
    operator Eigen::Matrix3d() const;
};

struct Mat_R {
    double data[3][3];
};

struct Mat_T {
    double data[3][1];
};

// 观测结构
struct Obser {
    int photoId;
    double x, y;
    Obser(int id = 0, double x = 0, double y = 0) : photoId(id), x(x), y(y) {}
};

// 特征点结构
struct TiePnt {
    Vec_t position;
    Color color;
    std::vector<Obser> observations;
};

// 照片结构
struct Photo {
    std::string label;
    Mat_R rotation;
    Mat_T translation;
    Vec_t position;
    Vec_t euler_angles;
    bool enabled;
};

// 确保这个定义存在
using Vec3 = Eigen::Vector3d;

} // namespace slam

#endif // COMMON_TYPES_HPP 