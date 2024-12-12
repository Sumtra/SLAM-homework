#ifndef IMAGE_PROCESSOR_HPP
#define IMAGE_PROCESSOR_HPP

#include <gdal_priv.h>
#include <string>
#include "../types/slam_types.hpp"
#include "../geometry/camera.hpp"

namespace slam {

class ImageProcessor {
public:
    static void CheckDataset(GDALDataset* dataset, const std::string& message);
    static GDALDataset* undistortImage(const std::string& imagePath, const Camera& camera);
    static void testUndistortion(const Camera& camera);
    
private:
    static void testCoordinateTransform(const Camera& camera);
};

}  // namespace slam
#endif // IMAGE_PROCESSOR_HPP 