#include "../../include/types/common_types.hpp"

namespace slam {

Mat_K::operator Eigen::Matrix3d() const {
    Eigen::Matrix3d K_eigen;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            K_eigen(i, j) = data[i][j];
        }
    }
    return K_eigen;
}

} // namespace slam 