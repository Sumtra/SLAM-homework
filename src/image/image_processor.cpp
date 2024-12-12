#include "../../include/image/image_processor.hpp"
#include "../../include/geometry/camera.hpp"
#include <iostream>
#include <cmath>

namespace slam {

void ImageProcessor::CheckDataset(GDALDataset* dataset, const std::string& message) {
    if (dataset == nullptr) {
        std::cerr << "错误: " << message << " - " << CPLGetLastErrorMsg() << std::endl;
        exit(EXIT_FAILURE);
    }
}

GDALDataset* ImageProcessor::undistortImage(const std::string& imagePath, const Camera& camera) {
    GDALAllRegister();

    // 打开输入图像
    GDALDataset* inputDS = static_cast<GDALDataset*>(GDALOpen(imagePath.c_str(), GA_ReadOnly));
    CheckDataset(inputDS, "无法读取图像: " + imagePath);

    int width = inputDS->GetRasterXSize();
    int height = inputDS->GetRasterYSize();
    int numBands = inputDS->GetRasterCount();
    std::cout << "图像尺寸: " << width << "x" << height << " 波段数: " << numBands << std::endl;

    // 创建输出图像
    GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("GTiff");
    std::string outPath = imagePath.substr(0, imagePath.find_last_of('.')) + "_undistorted.tif";
    GDALDataset* outputDS = driver->Create(outPath.c_str(), width, height, numBands, GDT_Byte, nullptr);
    CheckDataset(outputDS, "无法创建输出图像");

    // 为每个波段创建缓冲区
    std::vector<std::vector<uint8_t>> inputBuffers(numBands, std::vector<uint8_t>(width * height));
    std::vector<std::vector<uint8_t>> outputBuffers(numBands, std::vector<uint8_t>(width * height));

    // 读取所有波段数据
    for (int band = 1; band <= numBands; band++) {
        GDALRasterBand* inBand = inputDS->GetRasterBand(band);
        inBand->RasterIO(GF_Read, 0, 0, width, height,
            inputBuffers[band - 1].data(), width, height, GDT_Byte, 0, 0);
    }

    // 处理所有像素
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            // 计算畸变校正
            auto undistorted = camera.undistortPoint(x, y);
            int src_x = std::round(undistorted.first);
            int src_y = std::round(undistorted.second);

            // 对每个波段进行处理
            for (int band = 0; band < numBands; band++) {
                uint8_t value = 0;
                if (src_x >= 0 && src_x < width && src_y >= 0 && src_y < height) {
                    value = inputBuffers[band][src_y * width + src_x];
                }
                outputBuffers[band][y * width + x] = value;
            }
        }
    }

    // 写入所有波段数据
    for (int band = 1; band <= numBands; band++) {
        GDALRasterBand* outBand = outputDS->GetRasterBand(band);
        outBand->RasterIO(GF_Write, 0, 0, width, height,
            outputBuffers[band - 1].data(), width, height, GDT_Byte, 0, 0);
    }

    GDALClose(inputDS);
    GDALClose(outputDS);

    // 重新打开输出数据集以返回
    return static_cast<GDALDataset*>(GDALOpen(outPath.c_str(), GA_ReadOnly));
}

void ImageProcessor::testCoordinateTransform(const Camera& camera) {
    std::vector<std::pair<double, double>> test_points = {
        {100, 100},
        {500, 500},
        {1000, 1000}
    };

    std::cout << "\n=== 畸变测试结果 ===" << std::endl;
    for (const auto& point : test_points) {
        std::cout << "\n原始点: (" << point.first << ", " << point.second << ")" << std::endl;

        // 去畸变
        auto undist = camera.undistortPoint(point.first, point.second);
        std::cout << "去畸变后: (" << undist.first << ", " << undist.second << ")" << std::endl;

        // 重新添加畸变
        auto redist = camera.distortPoint(undist.first, undist.second);
        std::cout << "重新畸变后: (" << redist.first << ", " << redist.second << ")" << std::endl;

        // 计算误差
        double error_x = point.first - redist.first;
        double error_y = point.second - redist.second;
        double error = std::sqrt(error_x * error_x + error_y * error_y);
        std::cout << "误差: " << error << " 像素" << std::endl;
    }
}

void ImageProcessor::testUndistortion(const Camera& camera) {
    // 测试坐标转换
    testCoordinateTransform(camera);
    
    // 测试图像去畸变
    std::string imagePath = "test.jpg";  // 替换为实际的图像路径
    GDALDataset* undistorted = undistortImage(imagePath, camera);
    if (undistorted == nullptr) {
        std::cerr << "图像去畸变失败" << std::endl;
    }
    else {
        std::cout << "图像去畸变成功，输出路径: " << imagePath.substr(0, imagePath.find_last_of('.')) + "_undistorted.tif" << std::endl;
        GDALClose(undistorted);
    }
}

} // namespace slam