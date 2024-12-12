#ifndef SLAM_BLOCK_HPP
#define SLAM_BLOCK_HPP

#include <vector>
#include <string>
#include "../geometry/camera.hpp"
#include "../types/slam_types.hpp"

namespace slam {

    //class Camera;  // 前向声明
    // Block类外部辅助函数
    Vec_t rotationMatrixToEulerAngles(
        double r11, double r12, double r13,
        double r21, double r22, double r23,
        double r31, double r32, double r33);
    class Block {
    public:
        Camera camera;  // 相机成员
        std::vector<TiePnt> tiepoints;

        Block() = default;  // 默认构造函数

        bool loadFromXML(const std::string& filename);
        bool saveToXML(const std::string& filename) const;
    };

} // namespace slam

#endif // SLAM_BLOCK_HPP namespace slam {



