#include "../../include/types/block.hpp"
#include <iostream>
#include <tinyxml2.h>
#include <cmath>  // 添加这个用于atan2和sqrt函数
namespace slam {
    // 在Block类的实现之前添加函数定义
    Vec_t rotationMatrixToEulerAngles(
        double r11, double r12, double r13,
        double r21, double r22, double r23,
        double r31, double r32, double r33) {

        Vec_t angles;
        angles.x = atan2(r32, r33);
        angles.y = atan2(-r31, sqrt(r32 * r32 + r33 * r33));
        angles.z = atan2(r21, r11);
        return angles;
    }


// Block类XML读写实现
bool Block::loadFromXML(const std::string& filename) {
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(filename.c_str()) != tinyxml2::XML_SUCCESS) {
        std::cout << "XML文件加载失败: " << filename << std::endl;
        std::cout << "错误信息: " << doc.ErrorStr() << std::endl;
        return false;
    }

    tinyxml2::XMLElement* root = doc.RootElement();
    if (!root) return false;

    tinyxml2::XMLElement* blockElem = root->FirstChildElement("Block");
    if (!blockElem) return false;

    // 读取相机参数
    tinyxml2::XMLElement* photogroups = blockElem->FirstChildElement("Photogroups");
    if (photogroups) {
        tinyxml2::XMLElement* photogroup = photogroups->FirstChildElement("Photogroup");
        if (photogroup) {
            // 读取图像尺寸
            tinyxml2::XMLElement* imageDimensions = photogroup->FirstChildElement("ImageDimensions");
            if (imageDimensions) {
                tinyxml2::XMLElement* widthElem = imageDimensions->FirstChildElement("Width");
                tinyxml2::XMLElement* heightElem = imageDimensions->FirstChildElement("Height");
                // 可以存储尺寸信息如果需要的话
            }

            // 读取内参
            tinyxml2::XMLElement* focalLength = photogroup->FirstChildElement("FocalLength");
            double f = focalLength ? focalLength->DoubleText() : 0.0;

            // 读取主点
            tinyxml2::XMLElement* principalPoint = photogroup->FirstChildElement("PrincipalPoint");
            double cx = 0.0, cy = 0.0;
            if (principalPoint) {
                tinyxml2::XMLElement* x = principalPoint->FirstChildElement("x");
                tinyxml2::XMLElement* y = principalPoint->FirstChildElement("y");
                if (x) cx = x->DoubleText();
                if (y) cy = y->DoubleText();
            }

            // 设置相机内参矩阵
            camera.intrinsic.data[0][0] = f;  // fx
            camera.intrinsic.data[0][1] = 0;
            camera.intrinsic.data[0][2] = cx;
            camera.intrinsic.data[1][0] = 0;
            camera.intrinsic.data[1][1] = f;  // fy
            camera.intrinsic.data[1][2] = cy;
            camera.intrinsic.data[2][0] = 0;
            camera.intrinsic.data[2][1] = 0;
            camera.intrinsic.data[2][2] = 1;

            // 读取畸变系数
            tinyxml2::XMLElement* distortion = photogroup->FirstChildElement("Distortion");
            if (distortion) {
                tinyxml2::XMLElement* k1Elem = distortion->FirstChildElement("K1");
                tinyxml2::XMLElement* k2Elem = distortion->FirstChildElement("K2");
                tinyxml2::XMLElement* p1Elem = distortion->FirstChildElement("P1");
                tinyxml2::XMLElement* p2Elem = distortion->FirstChildElement("P2");

                if (k1Elem) camera.k1 = k1Elem->DoubleText();
                if (k2Elem) camera.k2 = k2Elem->DoubleText();
                if (p1Elem) camera.p1 = p1Elem->DoubleText();
                if (p2Elem) camera.p2 = p2Elem->DoubleText();
            }

            // 读取照片数据
            for (tinyxml2::XMLElement* photo = photogroup->FirstChildElement("Photo");
                photo;
                photo = photo->NextSiblingElement("Photo")) {

                Photo photoData;

                // 读取图像路径
                tinyxml2::XMLElement* imagePath = photo->FirstChildElement("ImagePath");
                if (imagePath && imagePath->GetText()) {
                    std::string path = imagePath->GetText();
                    size_t lastSlash = path.find_last_of("/\\");
                    photoData.label = (lastSlash != std::string::npos) ? path.substr(lastSlash + 1) : path;
                }

                // 读取位姿
                tinyxml2::XMLElement* poseElem = photo->FirstChildElement("Pose");
                if (poseElem) {
                    // 读取中心点位置
                    tinyxml2::XMLElement* center = poseElem->FirstChildElement("Center");
                    if (center) {
                        tinyxml2::XMLElement* x = center->FirstChildElement("x");
                        tinyxml2::XMLElement* y = center->FirstChildElement("y");
                        tinyxml2::XMLElement* z = center->FirstChildElement("z");

                        if (x && y && z) {
                            photoData.position = Vec_t(
                                x->DoubleText(),
                                y->DoubleText(),
                                z->DoubleText()
                            );
                        }
                    }

                    // 读取旋转矩阵
                    tinyxml2::XMLElement* rotation = poseElem->FirstChildElement("Rotation");
                    if (rotation) {
                        const char* elements[] = {
                            "M_00", "M_01", "M_02",
                            "M_10", "M_11", "M_12",
                            "M_20", "M_21", "M_22"
                        };

                        for (int i = 0; i < 3; i++) {
                            for (int j = 0; j < 3; j++) {
                                tinyxml2::XMLElement* elem = rotation->FirstChildElement(elements[i * 3 + j]);
                                if (elem) {
                                    photoData.rotation.data[i][j] = elem->DoubleText();
                                }
                            }
                        }

                        // 计算欧拉角
                        photoData.euler_angles = rotationMatrixToEulerAngles(
                            photoData.rotation.data[0][0], photoData.rotation.data[0][1], photoData.rotation.data[0][2],
                            photoData.rotation.data[1][0], photoData.rotation.data[1][1], photoData.rotation.data[1][2],
                            photoData.rotation.data[2][0], photoData.rotation.data[2][1], photoData.rotation.data[2][2]
                        );
                    }
                }

                photoData.enabled = true;
                camera.photos.push_back(photoData);
            }
        }
    }

    // 读取特征点
    tinyxml2::XMLElement* tiePointsElem = blockElem->FirstChildElement("TiePoints");
    if (tiePointsElem) {
        for (tinyxml2::XMLElement* tiePoint = tiePointsElem->FirstChildElement("TiePoint");
            tiePoint;
            tiePoint = tiePoint->NextSiblingElement("TiePoint")) {

            TiePnt tp;

            // 读取位置
            tinyxml2::XMLElement* position = tiePoint->FirstChildElement("Position");
            if (position) {
                tinyxml2::XMLElement* x = position->FirstChildElement("x");
                tinyxml2::XMLElement* y = position->FirstChildElement("y");
                tinyxml2::XMLElement* z = position->FirstChildElement("z");

                if (x && y && z) {
                    tp.position = Vec_t(
                        x->DoubleText(),
                        y->DoubleText(),
                        z->DoubleText()
                    );
                }
            }

            // 读取颜色
            tinyxml2::XMLElement* color = tiePoint->FirstChildElement("Color");
            if (color) {
                tinyxml2::XMLElement* r = color->FirstChildElement("Red");
                tinyxml2::XMLElement* g = color->FirstChildElement("Green");
                tinyxml2::XMLElement* b = color->FirstChildElement("Blue");

                if (r && g && b) {
                    tp.color = Color(
                        r->DoubleText(),
                        g->DoubleText(),
                        b->DoubleText()
                    );
                }
            }

            // 读取观测点
            for (tinyxml2::XMLElement* measurement = tiePoint->FirstChildElement("Measurement");
                measurement;
                measurement = measurement->NextSiblingElement("Measurement")) {

                tinyxml2::XMLElement* photoId = measurement->FirstChildElement("PhotoId");
                tinyxml2::XMLElement* x = measurement->FirstChildElement("x");
                tinyxml2::XMLElement* y = measurement->FirstChildElement("y");

                if (photoId && x && y) {
                    tp.observations.push_back(Obser(
                        photoId->IntText(),
                        x->DoubleText(),
                        y->DoubleText()
                    ));
                }
            }

            tiepoints.push_back(tp);
        }
    }

    return true;
}

bool Block::saveToXML(const std::string& filename) const {
    tinyxml2::XMLDocument doc;

    // 创建根元素
    tinyxml2::XMLElement* root = doc.NewElement("ContextCapture");
    doc.InsertFirstChild(root);

    // 创建Block元素
    tinyxml2::XMLElement* blockElem = doc.NewElement("Block");
    root->InsertEndChild(blockElem);

    // 保存相机参数
    tinyxml2::XMLElement* photogroups = doc.NewElement("Photogroups");
    blockElem->InsertEndChild(photogroups);

    tinyxml2::XMLElement* photogroup = doc.NewElement("Photogroup");
    photogroups->InsertEndChild(photogroup);

    // 保存内参
    tinyxml2::XMLElement* focalLength = doc.NewElement("FocalLength");
    focalLength->SetText(camera.intrinsic.data[0][0]);
    photogroup->InsertEndChild(focalLength);

    // ... 保存其他参数 ...

    return doc.SaveFile(filename.c_str()) == tinyxml2::XML_SUCCESS;
}

} // namespace slam 