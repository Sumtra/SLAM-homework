#include "../include/types/slam_types.hpp"
#include "../include/geometry/rigid_transform.hpp"
#include "../include/geometry/essential_matrix.hpp"
#include "../include/image/image_processor.hpp"
#include "../include/geometry/pnp_solver.hpp"
#include "../include/types/block.hpp"
#include <iostream>
#include <locale>
#include <iomanip>

using namespace slam;
using namespace std;

void showMenu() {
    cout << "\n=== SLAM测试程序 ===" << endl;
    cout << "1. 数据结构测试" << endl;

    cout << "2. 刚体变换测试" << endl;
    cout << "3. 本质矩阵测试" << endl;
    cout << "4. 图像去畸变测试" << endl;
    cout << "5. DLT+P3P算法测试" << endl;
    cout << "0. 退出" << endl;
    cout << "请选择: ";
}

int main() {
    // 设置中文支持
    #ifdef _WIN32
        setlocale(LC_ALL, "Chinese_China.UTF-8");
    #else
        setlocale(LC_ALL, "zh_CN.UTF-8");
    #endif

    int choice;
    Block blockData;

    // 从XML文件加载数据
    if (!blockData.loadFromXML("E:/slam_work/work_1_high.xml")) {
        cout << "无法加载XML文件" << endl;
        return -1;
    }

    while (true) {
        showMenu();
        cin >> choice;

        switch (choice) {
            case 1: {
                // 数据结构测试
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
                }
                break;
            }
            case 2: {
                if (blockData.camera.photos.size() >= 2) {
                    // 声明变量存储计算结果
                    Eigen::Matrix3d R;
                    Eigen::Vector3d t;

                    cout << "\n=== 需求一：计算两张影像间的刚体变换参数 ===" << endl;
                    cout << "第一张影像参数：" << endl;
                    cout << "位置 t1: ("
                        << blockData.camera.photos[0].position.x << ", "
                        << blockData.camera.photos[0].position.y << ", "
                        << blockData.camera.photos[0].position.z << ")" << endl;
                    cout << "旋转矩阵 R1:" << endl;
                    for (int i = 0; i < 3; i++) {
                        cout << "    [";
                        for (int j = 0; j < 3; j++) {
                            cout << fixed << setprecision(6) << blockData.camera.photos[0].rotation.data[i][j];
                            if (j < 2) cout << ", ";
                        }
                        cout << "]" << endl;
                    }

                    cout << "\n第二张影像参数：" << endl;
                    cout << "位置 t2: ("
                        << blockData.camera.photos[1].position.x << ", "
                        << blockData.camera.photos[1].position.y << ", "
                        << blockData.camera.photos[1].position.z << ")" << endl;
                    cout << "旋转矩阵 R2:" << endl;
                    for (int i = 0; i < 3; i++) {
                        cout << "    [";
                        for (int j = 0; j < 3; j++) {
                            cout << fixed << setprecision(6) << blockData.camera.photos[1].rotation.data[i][j];
                            if (j < 2) cout << ", ";
                        }
                        cout << "]" << endl;
                    }

                    // 1. 计算相对位姿
                    RigidTransformSolver::calculateRelativePose(
                        blockData.camera.photos[0],
                        blockData.camera.photos[1],
                        R,
                        t
                    );

                    cout << "\n计算得到的相对变换参数：" << endl;
                    cout << "相对旋转矩阵 R:" << endl << R << endl;
                    cout << "相对平移向量 t:" << endl << t << endl;

                    cout << "\n=== 需求二：利用刚体变换参数计算第二张影像位姿 ===" << endl;
                    
                    // 2. 使用相对位姿计算第二张照片的绝对位姿
                    Photo calculatedPhoto;
                    RigidTransformSolver::calculateAbsolutePose(
                        blockData.camera.photos[0],  // 参考照片
                        R,
                        t,
                        calculatedPhoto  // 输出计算得到的照片位姿
                    );

                    cout << "计算得到的第二张影像位姿：" << endl;
                    cout << "位置: ("
                        << calculatedPhoto.position.x << ", "
                        << calculatedPhoto.position.y << ", "
                        << calculatedPhoto.position.z << ")" << endl;
                    cout << "旋转矩阵:" << endl;
                    for (int i = 0; i < 3; i++) {
                        cout << "    [";
                        for (int j = 0; j < 3; j++) {
                            cout << fixed << setprecision(6) << calculatedPhoto.rotation.data[i][j];
                            if (j < 2) cout << ", ";
                        }
                        cout << "]" << endl;
                    }

                    // 3. 计算误差
                    double transformError = RigidTransformSolver::validateTransform(
                        blockData.camera.photos[0],
                        blockData.camera.photos[1],
                        R,
                        t
                    );

                    double posError = sqrt(
                        pow(calculatedPhoto.position.x - blockData.camera.photos[1].position.x, 2) +
                        pow(calculatedPhoto.position.y - blockData.camera.photos[1].position.y, 2) +
                        pow(calculatedPhoto.position.z - blockData.camera.photos[1].position.z, 2)
                    );

                    cout << "\n=== 误差分析 ===" << endl;
                    cout << "位置误差: " << posError << " 米" << endl;
                    cout << "相对变换误差: " << transformError << endl;
                }
                break;
            }
            case 3: {
                // 本质矩阵测试
                if (blockData.camera.photos.size() >= 2 && !blockData.tiepoints.empty()) {
                    cout << "\n=== 本质矩阵分解结果 ===" << endl;

                    // 声明变量接收结果
                    Eigen::Matrix3d R;
                    Eigen::Vector3d t;

                    // 调用计算方法
                    EssentialMatrixSolver::computeEssentialMatrix(
                        blockData.camera,
                        blockData.tiepoints,
                        0,  // 第一张照片ID
                        1,  // 第二张照片ID
                        R,  // 输出旋转矩阵
                        t   // 输出平移向量
                    );

                    cout << "估计的旋转矩阵 R：" << endl << R << endl;
                    cout << "估计的平移向量 t：" << endl << t << endl;

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
                    t_true.normalize();

                    // 计算误差
                    double R_error = (R - R_true).norm();
                    double t_error = (t - t_true).norm();

                    cout << "\n=== 误差分析 ===" << endl;
                    cout << "旋转矩阵误差: " << R_error << endl;
                    cout << "平移向量误差: " << t_error << endl;
                }
                break;
            }
            case 4: {
                // 图像去畸变测试
                Camera camera = blockData.camera;
                // 设置相机参数
                int width = 4592;
                int height = 3056;
                double sensorSize = 24.0128643127628;
                double focalLengthMM = 16.2714;
                double aspectRatio = static_cast<double>(height) / width;
                double focalLengthPixels = focalLengthMM * (width / sensorSize);

                // 设置相机内参
                camera.intrinsic.data[0][0] = focalLengthPixels;
                camera.intrinsic.data[1][1] = focalLengthPixels * aspectRatio;
                camera.intrinsic.data[0][2] = 2262.94;
                camera.intrinsic.data[1][2] = 1487.36;

                // 设置畸变系数
                camera.k1 = -0.0578184;
                camera.k2 = 0.0794939;
                camera.p1 = -0.00109465;
                camera.p2 = 0.000479776;
                // 使用GDAL进行影像去畸变
              try {
                std::string inputPath = "E:/slam_work/home/photos/DSC00161.JPG";
                std::string outputPath = "E:/slam_work/home/photos/gdal_undistorted.tif";  // 建议使用.tif格式
    
                GDALDataset* result = camera.undistortImage(inputPath, outputPath);
                if (result) {
                    std::cout << "影像去畸变成功完成！" << std::endl;
                    GDALClose(result);
                }
                } catch (const std::exception& e) {
                    std::cerr << "去畸变过程出错: " << e.what() << std::endl;
                }

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

                // 测试坐标变换
                vector<pair<double, double>> test_points = {
                    {100, 100}, {500, 500}, {1000, 1000}
                };

                cout << "\n=== 畸变测试结果 ===" << endl;
                for (const auto& point : test_points) {
                    cout << "\n原始点: (" << point.first << ", " << point.second << ")" << endl;
                    
                    auto undist = camera.undistortPoint(point.first, point.second);
                    cout << "去畸变后: (" << undist.first << ", " << undist.second << ")" << endl;
                    
                    auto redist = camera.distortPoint(undist.first, undist.second);
                    cout << "重新畸变后: (" << redist.first << ", " << redist.second << ")" << endl;
                    
                    double error = sqrt(pow(point.first - redist.first, 2) + 
                                      pow(point.second - redist.second, 2));
                    cout << "误差: " << error << " 像素" << endl;
                }
                break;
            }
            case 5: {
                cout << "\n=== DLT和P3P算法测试 ===" << endl;
                
                // 准备数据：收集3D点和对应的2D观测值
                std::vector<Point3D> points3D;
                std::vector<Obser> observations;
                
                // 选择一张测试照片（例如第一张）
                int test_photo_id = 25;
                
                // 从特征点中收集数据
                for (const auto& tp : blockData.tiepoints) {
                    for (const auto& obs : tp.observations) {
                        if (obs.photoId == test_photo_id) {
                            points3D.push_back(Point3D(tp.position.x, tp.position.y, tp.position.z));
                            observations.push_back(obs);
                            break;
                        }
                    }
                    
                    // 收集足够的点后停止
                    if (points3D.size() >= 20) {  // 使用20个点进行测试
                        break;
                    }
                }
                
                if (points3D.size() < 6) {  // DLT需要至少6个点
                    cout << "错误：没有足够的对应点" << endl;
                    break;
                }
                
                // 在调用RANSAC之前添加数据检查
                std::cout << "输入数据检查:" << std::endl;
                std::cout << "3D点数量: " << points3D.size() << std::endl;
                std::cout << "观测点数量: " << observations.size() << std::endl;
                // 打印几个样本点
                for(int i = 0; i < std::min(5, (int)points3D.size()); i++) {
                    std::cout << "Point " << i << ": (" 
                              << points3D[i].x << "," << points3D[i].y << "," << points3D[i].z 
                              << ") -> (" << observations[i].x << "," << observations[i].y << ")" 
                              << std::endl;
                }
                
                // 1. 测试DLT算法
                cout << "\n1. 测试DLT算法:" << endl;
                Mat_R R_dlt;
                Vec_t t_dlt;
                if (PnPSolver::solveDLT(points3D, observations, blockData.camera.intrinsic, R_dlt, t_dlt)) {
                    cout << "DLT求解成功！" << endl;
                    cout << "计算得到的旋转矩阵:" << endl;
                    for (int i = 0; i < 3; i++) {
                        for (int j = 0; j < 3; j++) {
                            cout << R_dlt.data[i][j] << " ";
                        }
                        cout << endl;
                    }
                    cout << "计算得到的平移向量: " << t_dlt.x << ", " << t_dlt.y << ", " << t_dlt.z << endl;
                    
                    // 计算重投影误差
                    double error = PnPSolver::computeReprojectionError(
                        points3D, observations, blockData.camera.intrinsic, R_dlt, t_dlt);
                    cout << "重投影误差: " << error << " 像素" << endl;
                } else {
                    cout << "DLT求解失败" << endl;
                }
                
                // 2. 测试P3P算法
                cout << "\n2. 测试P3P算法:" << endl;
                if (points3D.size() >= 4) {  // P3P需要4个点（3个用于求解，1个用于验证）
                    Mat_R R_p3p;
                    Vec_t t_p3p;
                    if (PnPSolver::solveP3P(
                        std::vector<Point3D>(points3D.begin(), points3D.begin() + 4),
                        std::vector<Obser>(observations.begin(), observations.begin() + 4),
                        blockData.camera.intrinsic, R_p3p, t_p3p)) {
                        
                        cout << "P3P求解成功！" << endl;
                        cout << "计算得到的旋转矩阵:" << endl;
                        for (int i = 0; i < 3; i++) {
                            for (int j = 0; j < 3; j++) {
                                cout << R_p3p.data[i][j] << " ";
                            }
                            cout << endl;
                        }
                        cout << "计算得到的平移向量: " << t_p3p.x << ", " << t_p3p.y << ", " << t_p3p.z << endl;
                        
                        // 计算重投影误差
                        double error = PnPSolver::computeReprojectionError(
                            points3D, observations, blockData.camera.intrinsic, R_p3p, t_p3p);
                        cout << "重投影误差: " << error << " 像素" << endl;
                    } else {
                        cout << "P3P求解失败" << endl;
                    }
                }
                
                // 3. 测试RANSAC+DLT
                cout << "\n3. 测试RANSAC+DLT:" << endl;
                Mat_R R_ransac_dlt;
                Vec_t t_ransac_dlt;
                if (PnPSolver::ransacDLT(points3D, observations, blockData.camera.intrinsic, R_ransac_dlt, t_ransac_dlt)) {
                    cout << "RANSAC+DLT求解成功！" << endl;
                    // ... 输出结果 ...
                }
                
                // 4. 测试RANSAC+P3P
                cout << "\n4. 测试RANSAC+P3P:" << endl;
                Mat_R R_ransac_p3p;
                Vec_t t_ransac_p3p;
                if (PnPSolver::ransacP3P(points3D, observations, blockData.camera.intrinsic, R_ransac_p3p, t_ransac_p3p)) {
                    cout << "RANSAC+P3P求解成功！" << endl;
                    // ... 输出结果 ...
                }
                
                break;
            }
            case 0:
                return 0;
            default:
                cout << "无效选择" << endl;
        }
    }

    return 0;
} 