#include "../include/types/slam_types.hpp"
#include <iostream>
#include <iomanip>
#include <tinyxml2.h>

namespace slam {

    // Mat_K构造函数实现
    Mat_K::Mat_K() {
        data[0][0] = 1.0; data[0][1] = 0.0; data[0][2] = 0.0;
        data[1][0] = 0.0; data[1][1] = 1.0; data[1][2] = 0.0;
        data[2][0] = 0.0; data[2][1] = 0.0; data[2][2] = 1.0;
    }
    
} // namespace slam