#include <iostream>
#include <tinyxml2.h>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <numeric>  // 为 std::accumulate
/*第三方库中的opencv的版本为release版本，我一直使用debug版本在调试代码，所以舍弃opencv*/
//#include <opencv2/opencv.hpp>
#include <filesystem>
#include <gdal_priv.h>
#include <gdal_utils.h>
using namespace std;

using namespace tinyxml2;

// 定义三维点结构
struct Point3D {
    double x, y, z;
    Point3D(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
};

// 定义相机位姿结构
struct CameraPose {
    string label;          // 图像名称
    Point3D position;      // 相机位置
    Point3D rotation;      // 相机旋转（欧拉角）
    bool enabled;          // 相机是否启用
};

// 在 Point3D 结构后添加新的结构定义
struct Color {
    double r, g, b;
    Color(double r = 0, double g = 0, double b = 0) : r(r), g(g), b(b) {}
};

struct Measurement {
    int photoId;
    double x, y;
    Measurement(int id = 0, double x = 0, double y = 0) : photoId(id), x(x), y(y) {}
};

struct TiePoint {
    Point3D position;
    Color color;
    vector<Measurement> measurements;
};

// SLAM数据结构
struct Mat_K {
    double data[3][3];
    Mat_K() {
        // 默认内参矩阵
        data[0][0] = 1.0; data[0][1] = 0.0; data[0][2] = 0.0;
        data[1][0] = 0.0; data[1][1] = 1.0; data[1][2] = 0.0;
        data[2][0] = 0.0; data[2][1] = 0.0; data[2][2] = 1.0;
    }
};

struct Mat_R {
    double data[3][3];
};

struct Mat_T {
    double data[3][1];
};

struct Vec_t {
    double x, y, z;
    Vec_t(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}

    // 添加赋值运算符
    Vec_t& operator=(const Point3D& p) {
        x = p.x;
        y = p.y;
        z = p.z;
        return *this;
    }
};

struct Obser {
    int photoId;
    double x, y;
    Obser(int id = 0, double x = 0, double y = 0) : photoId(id), x(x), y(y) {}
};

struct TiePnt {
    Vec_t position;
    Color color;
    vector<Obser> observations;
};

struct Photo {
    string label;
    Mat_R rotation;
    Mat_T translation;
    Vec_t position;
    Vec_t euler_angles;
    bool enabled;
};

struct Camera {
    Mat_K intrinsic;
    vector<Photo> photos;
    double k1 = 0.0;
    double k2 = 0.0;
    double p1 = 0.0;
    double p2 = 0.0;

    // 声明成员函数
    pair<double, double> pixelToNormalized(double x, double y) const;
    pair<double, double> normalizedToPixel(double xn, double yn) const;
    pair<double, double> undistortPoint(double x, double y) const;
    pair<double, double> distortPoint(double x, double y) const;
    Eigen::Matrix3d calculateDLT(const vector<pair<Point3D, pair<double, double>>>& correspondences) const;
    pair<double, double> projectPointWithDLT(const Point3D& point, const Eigen::Matrix3d& H) const;
};

// 在Camera结构体外实现成员函数
pair<double, double> Camera::pixelToNormalized(double x, double y) const {
    double xn = (x - intrinsic.data[0][2]) / intrinsic.data[0][0];
    double yn = (y - intrinsic.data[1][2]) / intrinsic.data[1][1];
    return make_pair(xn, yn);
}

pair<double, double> Camera::normalizedToPixel(double xn, double yn) const {
    double x = xn * intrinsic.data[0][0] + intrinsic.data[0][2];
    double y = yn * intrinsic.data[1][1] + intrinsic.data[1][2];
    return make_pair(x, y);
}

Eigen::Matrix3d Camera::calculateDLT(const vector<pair<Point3D, pair<double, double>>>& correspondences) const {
    // 1. 数据归一化
    Eigen::Vector3d mean_3d = Eigen::Vector3d::Zero();
    Eigen::Vector2d mean_2d = Eigen::Vector2d::Zero();

    // 计算质心
    for (const auto& corr : correspondences) {
        mean_3d += Eigen::Vector3d(corr.first.x, corr.first.y, corr.first.z);
        mean_2d += Eigen::Vector2d(corr.second.first, corr.second.second);
    }
    mean_3d /= correspondences.size();
    mean_2d /= correspondences.size();

    // 计算缩放因子
    double scale_3d = 0, scale_2d = 0;
    for (const auto& corr : correspondences) {
        scale_3d += ((Eigen::Vector3d(corr.first.x, corr.first.y, corr.first.z) - mean_3d).norm());
        scale_2d += ((Eigen::Vector2d(corr.second.first, corr.second.second) - mean_2d).norm());
    }
    scale_3d = sqrt(2.0) * correspondences.size() / scale_3d;
    scale_2d = sqrt(2.0) * correspondences.size() / scale_2d;

    // 2. 构建A矩阵
    Eigen::MatrixXd A(correspondences.size() * 2, 9);

    for (size_t i = 0; i < correspondences.size(); i++) {
        // 归一化坐标
        Eigen::Vector3d X((correspondences[i].first.x - mean_3d(0)) * scale_3d,
            (correspondences[i].first.y - mean_3d(1)) * scale_3d,
            1.0);
        Eigen::Vector2d x((correspondences[i].second.first - mean_2d(0)) * scale_2d,
            (correspondences[i].second.second - mean_2d(1)) * scale_2d);

        // 填充A矩阵
        A.block<1, 3>(i * 2, 0) = X.transpose();
        A.block<1, 3>(i * 2, 3) = Eigen::Vector3d::Zero().transpose();
        A.block<1, 3>(i * 2, 6) = -x(0) * X.transpose();

        A.block<1, 3>(i * 2 + 1, 0) = Eigen::Vector3d::Zero().transpose();
        A.block<1, 3>(i * 2 + 1, 3) = X.transpose();
        A.block<1, 3>(i * 2 + 1, 6) = -x(1) * X.transpose();
    }

    // 3. SVD求解
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
    Eigen::VectorXd h = svd.matrixV().col(8);

    // 4. 重构H矩阵
    Eigen::Matrix3d H;
    H << h(0), h(1), h(2),
         h(3), h(4), h(5),
         h(6), h(7), h(8);

    // 5. 反归一化
    Eigen::Matrix3d T_2d = Eigen::Matrix3d::Identity();
    T_2d.block<2, 2>(0, 0) *= scale_2d;
    T_2d.block<2, 1>(0, 2) = -scale_2d * mean_2d;

    Eigen::Matrix3d T_3d = Eigen::Matrix3d::Identity();
    T_3d.block<2, 2>(0, 0) *= scale_3d;
    T_3d.block<2, 1>(0, 2) = -scale_3d * mean_3d.head<2>();

    H = T_2d.inverse() * H * T_3d;

    // 归一化H矩阵
    H /= H(2, 2);

    return H;
}

pair<double, double> Camera::projectPointWithDLT(
    const Point3D& point,
    const Eigen::Matrix3d& H) const {

    // 将3D点转换为齐次坐标
    Eigen::Vector3d X(point.x, point.y, 1.0);  // 注意这里使用z=1

    // 应用投影变换
    Eigen::Vector3d x = H * X;

    // 齐次坐标归一化
    if (abs(x(2)) > 1e-10) {
        double px = x(0) / x(2);
        double py = x(1) / x(2);

        // 暂时不应用相机内参，直接返回投影结果
        return make_pair(px, py);
    }

    return make_pair(-1.0, -1.0);
}

// 测试函数定义
void testDLTTransformMultiple(const Camera& camera, const vector<TiePnt>& tiepoints) {
    for (size_t photoId = 0; photoId < camera.photos.size(); photoId++) {
        // 收集对应点
        vector<pair<Point3D, pair<double, double>>> correspondences;

        // 收集对应点
        for (const auto& tp : tiepoints) {
            for (const auto& obs : tp.observations) {
                if (obs.photoId == photoId) {
                    if (isfinite(tp.position.x) && isfinite(tp.position.y) && isfinite(tp.position.z) &&
                        isfinite(obs.x) && isfinite(obs.y)) {
                        correspondences.push_back({
                            Point3D(tp.position.x, tp.position.y, tp.position.z),
                            {obs.x, obs.y}
                            });
                    }
                    break;
                }
            }
        }

        if (correspondences.size() < 6) {
            cout << "照片 #" << photoId << " 有效点对数量不足" << endl;
            continue;
        }

        cout << "\n=== 照片 #" << photoId << " ===" << endl;

        // 输出旋转矩阵
        cout << "旋转矩阵：" << endl;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                cout << fixed << setprecision(6) << camera.photos[photoId].rotation.data[i][j] << " ";
            }
            cout << endl;
        }

        // 计算DLT矩阵
        Eigen::Matrix3d H = camera.calculateDLT(correspondences);

        // 计算重投影误差
        vector<double> errors;
        for (const auto& corr : correspondences) {
            auto projected = camera.projectPointWithDLT(corr.first, H);

            if (projected.first >= 0 && projected.second >= 0) {
                double error = sqrt(
                    pow(projected.first - corr.second.first, 2) +
                    pow(projected.second - corr.second.second, 2)
                );

                if (error < 1000) {
                    errors.push_back(error);
                }
            }
        }

        if (!errors.empty()) {
            double total_error = accumulate(errors.begin(), errors.end(), 0.0);
            double mean_error = total_error / errors.size();

            cout << "DLT重投影误差：" << endl;
            cout << "平均误差: " << mean_error << " 像素" << endl;
            cout << "有效点数: " << errors.size() << "/" << correspondences.size() << endl;
        }
        else {
            cout << "无有效重投影点" << endl;
        }
    }
}

struct Block {
    Camera camera;
    vector<TiePnt> tiepoints;
};

// 字符分割函数
vector<double> splitString(const string& str) {
    vector<double> result;
    stringstream ss(str);
    double value;
    while (ss >> value) {
        result.push_back(value);
    }
    return result;
}

// 从旋转矩阵算欧拉角
Vec_t rotationMatrixToEulerAngles(double r00, double r01, double r02,
    double r10, double r11, double r12,
    double r20, double r21, double r22) {
    Vec_t euler;

    // 计算欧拉角 (按照 ZYX 顺序)
    euler.y = asin(r02);  // pitch
    euler.x = atan2(-r12, r22);  // roll
    euler.z = atan2(-r01, r00);  // yaw

    return euler;
}

// 添加刚体变换参数结构体
struct RigidTransform {
    Eigen::Matrix3d R;  // 旋转矩阵
    Eigen::Vector3d t;  // 平移向量

    RigidTransform() {
        R.setIdentity();
        t.setZero();
    }
};

// 计算两张影像之间的刚体变换参数
RigidTransform calculateRigidTransform(const Photo& photo1, const Photo& photo2) {
    RigidTransform transform;

    // 将Mat_R转换为Eigen::Matrix3d
    Eigen::Matrix3d R1, R2;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R1(i, j) = photo1.rotation.data[i][j];
            R2(i, j) = photo2.rotation.data[i][j];
        }
    }

    // 将Vec_t转换为Eigen::Vector3d
    Eigen::Vector3d t1(photo1.position.x, photo1.position.y, photo1.position.z);
    Eigen::Vector3d t2(photo2.position.x, photo2.position.y, photo2.position.z);

    // 算相对旋转矩阵：R12 = R2 * R1.transpose()
    transform.R = R2 * R1.transpose();

    // 计算相对平移向量：t12 = t2 - R12 * t1
    transform.t = t2 - transform.R * t1;

    return transform;
}

// 使用刚体变换参数计算另一张影像的位姿
Photo calculateNewPose(const Photo& photo1, const RigidTransform& transform) {
    Photo newPhoto;

    // 将Mat_R转换为Eigen::Matrix3d
    Eigen::Matrix3d R1;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R1(i, j) = photo1.rotation.data[i][j];
        }
    }

    // 将Vec_t转换为Eigen::Vector3d
    Eigen::Vector3d t1(photo1.position.x, photo1.position.y, photo1.position.z);

    // 计算新的旋转矩阵：R2 = R12 * R1
    Eigen::Matrix3d R2 = transform.R * R1;

    // 计算新的平移向量：t2 = R12 * t1 + t12
    Eigen::Vector3d t2 = transform.R * t1 + transform.t;

    // 将结果存回Photo结构
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            newPhoto.rotation.data[i][j] = R2(i, j);
        }
        newPhoto.translation.data[i][0] = t2(i);
    }

    newPhoto.position = Vec_t(t2.x(), t2.y(), t2.z());
    newPhoto.euler_angles = rotationMatrixToEulerAngles(
        R2(0, 0), R2(0, 1), R2(0, 2),
        R2(1, 0), R2(1, 1), R2(1, 2),
        R2(2, 0), R2(2, 1), R2(2, 2)
    );

    return newPhoto;
}

// 计算本质矩阵和SVD分解的函数
struct EssentialMatrixResult {
    Eigen::Matrix3d E;  // 本质矩阵
    Eigen::Matrix3d R;  // 旋转矩阵
    Eigen::Vector3d t;  // 平移向量
};

EssentialMatrixResult calculateEssentialMatrix(
    const Photo& photo1,
    const Photo& photo2,
    const vector<TiePnt>& tiepoints,
    const Mat_K& K) {

    EssentialMatrixResult result;
    vector<pair<Eigen::Vector2d, Eigen::Vector2d>> matches;

    // 收集匹配点
    for (const auto& tp : tiepoints) {
        Eigen::Vector2d pt1, pt2;
        bool found1 = false, found2 = false;

        for (const auto& obs : tp.observations) {
            if (obs.photoId == 0) {  // 假设第一张图片ID为0
                pt1 = Eigen::Vector2d(obs.x, obs.y);
                found1 = true;
            }
            if (obs.photoId == 1) {  // 假设第二张图片ID为1
                pt2 = Eigen::Vector2d(obs.x, obs.y);
                found2 = true;
            }
        }

        if (found1 && found2) {
            matches.push_back({ pt1, pt2 });
        }
    }

    // 构建系数矩阵A
    Eigen::MatrixXd A(matches.size(), 9);

    // 转换K矩阵为Eigen格式
    Eigen::Matrix3d K_eigen;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            K_eigen(i, j) = K.data[i][j];
        }
    }
    Eigen::Matrix3d K_inv = K_eigen.inverse();

    for (size_t i = 0; i < matches.size(); i++) {
        // 归一化坐标
        Eigen::Vector3d p1 = K_inv * Eigen::Vector3d(matches[i].first.x(), matches[i].first.y(), 1.0);
        Eigen::Vector3d p2 = K_inv * Eigen::Vector3d(matches[i].second.x(), matches[i].second.y(), 1.0);

        A(i, 0) = p2.x() * p1.x();
        A(i, 1) = p2.x() * p1.y();
        A(i, 2) = p2.x() * p1.z();
        A(i, 3) = p2.y() * p1.x();
        A(i, 4) = p2.y() * p1.y();
        A(i, 5) = p2.y() * p1.z();
        A(i, 6) = p2.z() * p1.x();
        A(i, 7) = p2.z() * p1.y();
        A(i, 8) = p2.z() * p1.z();
    }

    // SVD分解求解本质矩阵
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
    Eigen::VectorXd e = svd.matrixV().col(8);
    result.E = Eigen::Map<Eigen::Matrix3d>(e.data());

    // 对本质矩阵进行SVD分解
    Eigen::JacobiSVD<Eigen::Matrix3d> svd_E(result.E, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd_E.matrixU();
    Eigen::Matrix3d V = svd_E.matrixV();

    // 构造W矩阵
    Eigen::Matrix3d W;
    W << 0, -1, 0,
        1, 0, 0,
        0, 0, 1;

    // 计算可能的R和t
    result.R = U * W * V.transpose();
    result.t = U.col(2);

    // 确保R是正交矩阵且行列式为1
    if (result.R.determinant() < 0) {
        result.R = -result.R;
        result.t = -result.t;
    }

    return result;
}

void CheckDataset(GDALDataset* dataset, const string& message) {
    if (dataset == nullptr) {
        cerr << "错误: " << message << " - " << CPLGetLastErrorMsg() << endl;
        exit(EXIT_FAILURE);
    }
}

GDALDataset* undistortImage(const string& imagePath, const Camera& camera) {
    GDALAllRegister();

    // 打开输入图像
    GDALDataset* inputDS = (GDALDataset*)GDALOpen(imagePath.c_str(), GA_ReadOnly);
    CheckDataset(inputDS, "无法读取图像: " + imagePath);

    int width = inputDS->GetRasterXSize();
    int height = inputDS->GetRasterYSize();
    int numBands = inputDS->GetRasterCount();  // 获取波段数量
    cout << "图像尺寸: " << width << "x" << height << " 波段数: " << numBands << endl;

    // 创建输出图像
    GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("GTiff");
    string outPath = imagePath.substr(0, imagePath.find_last_of('.')) + "_undistorted.tif";
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
            double xd = (x - camera.intrinsic.data[0][2]) / camera.intrinsic.data[0][0];
            double yd = (y - camera.intrinsic.data[1][2]) / camera.intrinsic.data[1][1];

            // 使用迭代法求解无畸变坐标
            double xn = xd, yn = yd;
            for (int i = 0; i < 5; i++) {
                double r2 = xn * xn + yn * yn;
                double radial = 1 + camera.k1 * r2 + camera.k2 * r2 * r2;
                double dx = 2 * camera.p1 * xn * yn + camera.p2 * (r2 + 2 * xn * xn);
                double dy = camera.p1 * (r2 + 2 * yn * yn) + 2 * camera.p2 * xn * yn;

                double xn_new = (xd - dx) / radial;
                double yn_new = (yd - dy) / radial;

                xn = xn_new;
                yn = yn_new;
            }

            int src_x = round(xn * camera.intrinsic.data[0][0] + camera.intrinsic.data[0][2]);
            int src_y = round(yn * camera.intrinsic.data[1][1] + camera.intrinsic.data[1][2]);

            // 对每个波段进处理
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
    return (GDALDataset*)GDALOpen(outPath.c_str(), GA_ReadOnly);
}

//手敲算法实现

/*检查xml格式*/
// 函数声明
void printXMLStructure(XMLElement* element, int depth = 0, bool* foundFirstTiePoint = nullptr) {
    static bool defaultFlag = false;
    bool& flag = foundFirstTiePoint ? *foundFirstTiePoint : defaultFlag;

    while (element && !flag) {
        // 检查是否是第二个TiePoint
        if (strcmp(element->Name(), "TiePoint") == 0) {
            if (flag) {
                return;
            }
            flag = true;
        }

        // 打印缩进
        for (int i = 0; i < depth; i++) cout << "  ";

        // 打印元素名称
        cout << element->Name();

        // 如果有值，打印值
        if (element->GetText())
            cout << ": " << element->GetText();
        cout << endl;

        // 递归处理子元素
        printXMLStructure(element->FirstChildElement(), depth + 1, &flag);
        element = element->NextSiblingElement();
    }
}

int main() {
    XMLDocument doc;
    Block blockData;

    // 加载XML文件
    XMLError result = doc.LoadFile("E:/slam_work/work_1_high.xml");
    if (result != XML_SUCCESS) {
        cout << "无法加载XML文件！错误代码: " << result << endl;
        return -1;
    }

    // 获取根元素
    XMLElement* root = doc.RootElement();
    if (!root) {
        cout << "无法找到根元素" << endl;
        return -1;
    }

    // 获取Block元素
    XMLElement* blockElem = root->FirstChildElement("Block");
    if (!blockElem) {
        cout << "无法找到Block元素" << endl;
        return -1;
    }

    // 获取Photogroups元素
    XMLElement* photogroups = blockElem->FirstChildElement("Photogroups");
    if (!photogroups) {
        cout << "无法找到Photogroups元素" << endl;
        return -1;
    }

    // 获取第一个Photogroup元素，内参读取
    XMLElement* photogroup = photogroups->FirstChildElement("Photogroup");
    if (photogroup) {
        // 读取图像尺寸
        XMLElement* imageDimensions = photogroup->FirstChildElement("ImageDimensions");
        int width = 0, height = 0;
        if (imageDimensions) {
            XMLElement* widthElem = imageDimensions->FirstChildElement("Width");
            XMLElement* heightElem = imageDimensions->FirstChildElement("Height");
            if (widthElem) width = widthElem->IntText();
            if (heightElem) height = heightElem->IntText();
        }

        // 读取焦距
        XMLElement* focalLength = photogroup->FirstChildElement("FocalLength");
        double f = focalLength ? focalLength->DoubleText() : 0.0;

        // 读取主点坐标
        XMLElement* principalPoint = photogroup->FirstChildElement("PrincipalPoint");
        double cx = 0.0, cy = 0.0;
        if (principalPoint) {
            XMLElement* x = principalPoint->FirstChildElement("x");
            XMLElement* y = principalPoint->FirstChildElement("y");
            if (x) cx = x->DoubleText();
            if (y) cy = y->DoubleText();
        }

        // 读取畸变系数
        XMLElement* distortion = photogroup->FirstChildElement("Distortion");
        if (distortion) {
            XMLElement* k1Elem = distortion->FirstChildElement("K1");
            XMLElement* k2Elem = distortion->FirstChildElement("K2");
            XMLElement* p1Elem = distortion->FirstChildElement("P1");
            XMLElement* p2Elem = distortion->FirstChildElement("P2");

            if (k1Elem) blockData.camera.k1 = k1Elem->DoubleText();
            if (k2Elem) blockData.camera.k2 = k2Elem->DoubleText();
            if (p1Elem) blockData.camera.p1 = p1Elem->DoubleText();
            if (p2Elem) blockData.camera.p2 = p2Elem->DoubleText();
        }

        // 设置相机内参矩阵
        blockData.camera.intrinsic.data[0][0] = f;  // fx
        blockData.camera.intrinsic.data[0][1] = 0;
        blockData.camera.intrinsic.data[0][2] = cx; // cx

        blockData.camera.intrinsic.data[1][0] = 0;
        blockData.camera.intrinsic.data[1][1] = f;  // fy
        blockData.camera.intrinsic.data[1][2] = cy; // cy

        blockData.camera.intrinsic.data[2][0] = 0;
        blockData.camera.intrinsic.data[2][1] = 0;
        blockData.camera.intrinsic.data[2][2] = 1;
        /*
        // 打印读取的参数进行查验
        cout << "相机参数：" << endl;
        cout << "图像尺寸: " << width << "x" << height << endl;
        cout << "焦距: " << f << endl;
        cout << "主点: (" << cx << ", " << cy << ")" << endl;
        cout << "畸变系数: " << endl;
        cout << "k1: " << blockData.camera.k1 << endl;
        cout << "k2: " << blockData.camera.k2 << endl;
        cout << "p1: " << blockData.camera.p1 << endl;
        cout << "p2: " << blockData.camera.p2 << endl;

        cout << "\n相机内参矩阵：" << endl;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                cout << blockData.camera.intrinsic.data[i][j] << " ";
            }
            cout << endl;
        }*/
    }

    // 遍历所有Photo元素
    for (XMLElement* photo = photogroup->FirstChildElement("Photo");
        photo;
        photo = photo->NextSiblingElement("Photo")) {

        Photo photoData;

        // 获取图像路径
        XMLElement* imagePath = photo->FirstChildElement("ImagePath");
        if (imagePath && imagePath->GetText()) {
            string path = imagePath->GetText();
            size_t lastSlash = path.find_last_of("/\\");
            photoData.label = (lastSlash != string::npos) ? path.substr(lastSlash + 1) : path;
        }

        // 获取Pose元素
        XMLElement* poseElem = photo->FirstChildElement("Pose");
        if (poseElem) {
            // 读取Center位置
            XMLElement* center = poseElem->FirstChildElement("Center");
            if (center) {
                XMLElement* x = center->FirstChildElement("x");
                XMLElement* y = center->FirstChildElement("y");
                XMLElement* z = center->FirstChildElement("z");

                if (x && y && z) {
                    photoData.position = Vec_t(
                        x->DoubleText(),
                        y->DoubleText(),
                        z->DoubleText()
                    );
                }
            }

            // 读取Rotation矩阵
            XMLElement* rotation = poseElem->FirstChildElement("Rotation");
            if (rotation) {
                // 读取旋转矩元
                const char* elements[] = {
                    "M_00", "M_01", "M_02",
                    "M_10", "M_11", "M_12",
                    "M_20", "M_21", "M_22"
                };

                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 3; j++) {
                        XMLElement* elem = rotation->FirstChildElement(elements[i * 3 + j]);
                        if (elem) {
                            photoData.rotation.data[i][j] = elem->DoubleText();
                        }
                    }
                }

                // 计算欧拉
                photoData.euler_angles = rotationMatrixToEulerAngles(
                    photoData.rotation.data[0][0], photoData.rotation.data[0][1], photoData.rotation.data[0][2],
                    photoData.rotation.data[1][0], photoData.rotation.data[1][1], photoData.rotation.data[1][2],
                    photoData.rotation.data[2][0], photoData.rotation.data[2][1], photoData.rotation.data[2][2]
                );
            }
        }

        photoData.enabled = true;
        blockData.camera.photos.push_back(photoData);
    }

    // 处理特征点
    XMLElement* tiePointsElem = blockElem->FirstChildElement("TiePoints");
    if (tiePointsElem) {
        for (XMLElement* tiePoint = tiePointsElem->FirstChildElement("TiePoint");
            tiePoint;
            tiePoint = tiePoint->NextSiblingElement("TiePoint")) {

            TiePnt tp;

            // 读取位置
            XMLElement* position = tiePoint->FirstChildElement("Position");
            if (position) {
                XMLElement* x = position->FirstChildElement("x");
                XMLElement* y = position->FirstChildElement("y");
                XMLElement* z = position->FirstChildElement("z");

                if (x && y && z) {
                    tp.position = Vec_t(
                        x->DoubleText(),
                        y->DoubleText(),
                        z->DoubleText()
                    );
                }
            }

            // 读取颜色
            XMLElement* color = tiePoint->FirstChildElement("Color");
            if (color) {
                XMLElement* r = color->FirstChildElement("Red");
                XMLElement* g = color->FirstChildElement("Green");
                XMLElement* b = color->FirstChildElement("Blue");

                if (r && g && b) {
                    tp.color = Color(
                        r->DoubleText(),
                        g->DoubleText(),
                        b->DoubleText()
                    );
                }
            }

            // 读取观测点
            for (XMLElement* measurement = tiePoint->FirstChildElement("Measurement");
                measurement;
                measurement = measurement->NextSiblingElement("Measurement")) {

                XMLElement* photoId = measurement->FirstChildElement("PhotoId");
                XMLElement* x = measurement->FirstChildElement("x");
                XMLElement* y = measurement->FirstChildElement("y");

                if (photoId && x && y) {
                    tp.observations.push_back(Obser(
                        photoId->IntText(),
                        x->DoubleText(),
                        y->DoubleText()
                    ));
                }
            }

            blockData.tiepoints.push_back(tp);
        }
    }
    /*输出xml文件格式检查*/
    /*
    // 在main函数中调用
    XMLElement* rot = doc.RootElement();
    if (rot) {
        cout << "XML文件结构：" << endl;
        printXMLStructure(rot);
    }
    return 0;
}*/
//一
/*
   //第一题输出

   // 输出所有照片信息
   cout << "\n=== 照片信息 ===" << endl;
   for (const auto& photo : blockData.camera.photos) {
       cout << "\n图像名称: " << photo.label << endl;
       cout << "位置: (" << photo.position.x << ", "
           << photo.position.y << ", "
           << photo.position.z << ")" << endl;
       cout << "欧拉角(弧度): (" << photo.euler_angles.x << ", "
           << photo.euler_angles.y << ", "
           << photo.euler_angles.z << ")" << endl;
       cout << "旋转矩阵:" << endl;
       for (int i = 0; i < 3; i++) {
           cout << "[";
           for (int j = 0; j < 3; j++) {
               cout << fixed << setprecision(6) << photo.rotation.data[i][j];
               if (j < 2) cout << ", ";
           }
           cout << "]" << endl;
       }
   }

   // 输出前20个特征点信息
   cout << "\n=== 特征点信息（前20个） ===" << endl;
   int count = 0;
   for (const auto& tp : blockData.tiepoints) {
       if (count >= 20) break;

       cout << "\n特征点 #" << count + 1 << endl;
       cout << "位置: (" << tp.position.x << ", "
           << tp.position.y << ", "
           << tp.position.z << ")" << endl;
       cout << "颜色 (RGB): (" << tp.color.r << ", "
           << tp.color.g << ", "
           << tp.color.b << ")" << endl;
       cout << "观测数量: " << tp.observations.size() << endl;

       count++;
       return 0;
   }
}
*/
//二
/*第二题输出
// 测试刚体变换计算
if (blockData.camera.photos.size() >= 2) {
    // 计算第一张和第二张照片之间的刚体变换
    RigidTransform transform = calculateRigidTransform(
        blockData.camera.photos[0],
        blockData.camera.photos[1]
    );

    // 使用第一张照片和变换数计算第二张照片的位姿
    Photo calculatedPhoto = calculateNewPose(
        blockData.camera.photos[0],
        transform
    );

    // 输出详细的比结果
    cout << "\n=== 原始第二张照片参数 ===" << endl;
    cout << "位置 (X,Y,Z): "
        << blockData.camera.photos[1].position.x << ", "
        << blockData.camera.photos[1].position.y << ", "
        << blockData.camera.photos[1].position.z << endl;

    cout << "旋转矩阵：" << endl;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            cout << blockData.camera.photos[1].rotation.data[i][j] << " ";
        }
        cout << endl;
    }

    cout << "欧拉角 (弧度): "
        << blockData.camera.photos[1].euler_angles.x << ", "
        << blockData.camera.photos[1].euler_angles.y << ", "
        << blockData.camera.photos[1].euler_angles.z << endl;

    cout << "\n=== 计算得到的第二张照片参数 ===" << endl;
    cout << "位置 (X,Y,Z): "
        << calculatedPhoto.position.x << ", "
        << calculatedPhoto.position.y << ", "
        << calculatedPhoto.position.z << endl;

    cout << "旋转矩阵：" << endl;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            cout << calculatedPhoto.rotation.data[i][j] << " ";
        }
        cout << endl;
    }

    cout << "欧拉角 (弧度): "
        << calculatedPhoto.euler_angles.x << ", "
        << calculatedPhoto.euler_angles.y << ", "
        << calculatedPhoto.euler_angles.z << endl;

    // 输出刚体变换参数
    cout << "\n=== 计算得到的刚体变换参数 ===" << endl;
    cout << "旋转矩阵 R12：" << endl;
    cout << transform.R << endl;

    cout << "平移向量 t12：" << endl;
    cout << transform.t.transpose() << endl;

    // 计算误差
    double position_error = sqrt(
        pow(blockData.camera.photos[1].position.x - calculatedPhoto.position.x, 2) +
        pow(blockData.camera.photos[1].position.y - calculatedPhoto.position.y, 2) +
        pow(blockData.camera.photos[1].position.z - calculatedPhoto.position.z, 2)
    );

    cout << "\n=== 误差分析 ===" << endl;
    cout << "位置误差（欧氏距离）: " << position_error << endl;
}
return 0;
}

*/
//三
/*
// 第三题输出
if (blockData.camera.photos.size() >= 2 && !blockData.tiepoints.empty()) {
    cout << "\n=== 本质矩阵分解果 ===" << endl;

    EssentialMatrixResult result = calculateEssentialMatrix(
        blockData.camera.photos[0],
        blockData.camera.photos[1],
        blockData.tiepoints,
        blockData.camera.intrinsic
    );

    cout << "本质矩阵 E：" << endl << result.E << endl;
    cout << "\n估计的旋转矩阵 R：" << endl << result.R << endl;
    cout << "\n估计的平移向量 t：" << endl << result.t << endl;

    // 计算与真实值的误差
    Eigen::Matrix3d R_true;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R_true(i, j) = blockData.camera.photos[1].rotation.data[i][j];
        }
    }

    Eigen::Vector3d t_true(
        blockData.camera.photos[1].position.x - blockData.camera.photos[0].position.x,
        blockData.camera.photos[1].position.y - blockData.camera.photos[0].position.y,
        blockData.camera.photos[1].position.z - blockData.camera.photos[0].position.z
    );
    t_true.normalize();  // 归一化以便比较

    double R_error = (result.R - R_true).norm();
    double t_error = (result.t - t_true).norm();

    cout << "\n=== 误差分析 ===" << endl;
    cout << "旋转矩阵误差: " << R_error << endl;
    cout << "平移向量误差: " << t_error << endl;
}
return 0;
}

*/

//四

// 在main函数中使用
    Camera camera = { blockData.camera };

    // 从输出中获取图像尺寸
    int width = 4592;
    int height = 3056;

    // 计算焦距（从毫米转换为像素）
    double sensorSize = 24.0128643127628;  // 从XML中获取  传感器尺寸
    double focalLengthMM = 16.2714;
    double aspectRatio = static_cast<double>(height) / width;
    double focalLengthPixels = focalLengthMM * (width / sensorSize);

    // 设置相机内参
    camera.intrinsic.data[0][0] = focalLengthPixels;  // fx
    camera.intrinsic.data[1][1] = focalLengthPixels * aspectRatio;  // fy
    camera.intrinsic.data[0][1] = 0;
    camera.intrinsic.data[0][2] = 2262.94;  // cx

    camera.intrinsic.data[1][0] = 0;
    camera.intrinsic.data[1][2] = 1487.36;  // cy

    camera.intrinsic.data[2][0] = 0;
    camera.intrinsic.data[2][1] = 0;
    camera.intrinsic.data[2][2] = 1;

    // 设畸变系数
    camera.k1 = -0.0578184;
    camera.k2 = 0.0794939;
    camera.p1 = -0.00109465;
    camera.p2 = 0.000479776;
    /*输出基本参数验证*/
    /*
    // 打印参数验证
    cout << "\n=== 去畸变参数验证 ===" << endl;
    cout << "图像尺寸: " << width << "x" << height << endl;
    cout << "焦距(像素): " << focalLengthPixels << endl;
    cout << "相机内参矩阵：" << endl;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            cout << camera.intrinsic.data[i][j] << " ";
        }
        cout << endl;
    }
    cout << "畸变系数：" << endl;
    cout << "k1: " << camera.k1 << endl;
    cout << "k2: " << camera.k2 << endl;
    cout << "p1: " << camera.p1 << endl;
    cout << "p2: " << camera.p2 << endl;
    */
    /*4—1*/
    /*
    // 去畸变处理
    string imagePath = "E:/slam_work/home/photos/DSC00164.JPG";  // 图片路径
    GDALDataset* undistorted = undistortImage(imagePath, camera);
    if (undistorted == nullptr) {
        cerr << "图像去畸变失败" << endl;
    }
    else {
        cout << "图像去畸变成功，输出路径: " << imagePath.substr(0, imagePath.find_last_of('.')) + "_undistorted.tif" << endl;
        GDALClose(undistorted);
    }
    */
    /*4-2*///手敲算法实现去畸变的正反过程
    /*
    // 测试几个点的去畸变和重新畸变
    vector<pair<double, double>> test_points = {
        {100, 100},
        {500, 500},
        {1000, 1000}
    };

    cout << "\n=== 畸变测试结果 ===" << endl;
    for (const auto& point : test_points) {
        cout << "\n原始点: (" << point.first << ", " << point.second << ")" << endl;

        // 去畸变
        pair<double, double> undist = camera.undistortPoint(point.first, point.second);
        cout << "去畸变后: (" << undist.first << ", " << undist.second << ")" << endl;

        // 重新添加畸变
        pair<double, double> redist = camera.distortPoint(undist.first, undist.second);
        cout << "重新畸变后: (" << redist.first << ", " << redist.second << ")" << endl;

        // 计算误差
        double error_x = point.first - redist.first;
        double error_y = point.second - redist.second;
        double error = sqrt(error_x * error_x + error_y * error_y);
        cout << "误差: " << error << " 像素" << endl;
    }

    return 0;
}
    */



    /*实验5_1 直接线性变换*/
    // 测试DLT变换
    testDLTTransformMultiple(blockData.camera, blockData.tiepoints);

    return 0;
}