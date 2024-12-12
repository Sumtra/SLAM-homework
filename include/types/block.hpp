#ifndef SLAM_BLOCK_HPP
#define SLAM_BLOCK_HPP

#include <vector>
#include <string>
#include "common_types.hpp"
#include "../../include/geometry/camera.hpp"
namespace slam {

//class Camera;  // 前向声明

class Block {
public:
    Camera camera;  // 改回非指针成员
    std::vector<TiePnt> tiepoints;

    Block() = default;  // 添加默认构造函数
    
    bool loadFromXML(const std::string& filename);
    bool saveToXML(const std::string& filename) const;
};

} // namespace slam

#endif // SLAM_BLOCK_HPP 