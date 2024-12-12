from src.core.block import Block
from src.structures.vectors import Vec_t
from src.structures.matrices import Mat_R
import locale
import sys
import os
from typing import List

def show_menu():
    """显示菜单选项"""
    print("\n=== SLAM数据结构测试程序 ===")
    print("1. 显示数据结构信息")
    print("2. 位姿估计 (DLT)")
    print("3. 位姿估计 (P3P)")
    print("4. 位姿估计 (RANSAC + DLT)")
    print("5. 位姿估计 (RANSAC + P3P)")
    print("0. 退出")
    print("请选择操作: ", end='')

def display_photo_info(photos: List):
    """显示照片信息"""
    print("\n=== 照片信息 ===")
    for photo in photos:
        print(f"\n图像名称: {photo.label}")
        print(f"位置: ({photo.position.x:.6f}, {photo.position.y:.6f}, {photo.position.z:.6f})")
        print(f"欧拉角(弧度): ({photo.euler_angles.x:.6f}, {photo.euler_angles.y:.6f}, {photo.euler_angles.z:.6f})")
        print("旋转矩阵:")
        for i in range(3):
            row = photo.rotation.data[i]
            print(f"[{row[0]:10.6f}, {row[1]:10.6f}, {row[2]:10.6f}]")

def display_tiepoint_info(tiepoints: List, limit: int = 20):
    """显示特征点信息"""
    print(f"\n=== 特征点信息（前{limit}个） ===")
    for i, tp in enumerate(tiepoints[:limit]):
        print(f"\n特征点 #{i + 1}")
        print(f"位置: ({tp.position.x:.6f}, {tp.position.y:.6f}, {tp.position.z:.6f})")
        if tp.color:
            print(f"颜色 (RGB): ({tp.color.r}, {tp.color.g}, {tp.color.b})")
        print(f"观测数量: {len(tp.observations)}")

def display_pose_estimation_result(R: Mat_R, t: Vec_t, inliers: List[bool] = None, error: float = None):
    """显示位姿估计结果"""
    print("\n=== 位姿估计结果 ===")
    print("旋转矩阵:")
    for i in range(3):
        row = R.data[i]
        print(f"[{row[0]:10.6f}, {row[1]:10.6f}, {row[2]:10.6f}]")
    print(f"\n平移向量: ({t.x:.6f}, {t.y:.6f}, {t.z:.6f})")
    
    if error is not None:
        if error == float('inf'):
            print("重投影误差: 无效")
        else:
            print(f"平均重投影误差: {error:.3f}像素")
    
    if inliers is not None:
        inlier_count = sum(1 for x in inliers if x)
        print(f"内点数量: {inlier_count}/{len(inliers)} ({inlier_count/len(inliers)*100:.2f}%)")

def process_pose_estimation(block_data: Block, method: str):
    """处理位姿估计"""
    # 显示可用的照片
    print("\n可用的照片:")
    for i, photo in enumerate(block_data.camera.photos):
        print(f"{i}: {photo.label}")
    
    try:
        idx = int(input("\n请选择照片编号: "))
        if idx < 0 or idx >= len(block_data.camera.photos):
            print("无效的照片编号")
            return
            
        if method == 'dlt':
            result = block_data.estimate_pose_dlt(idx)
            if result:
                R, t, error = result
                display_pose_estimation_result(R, t, error=error)
        elif method == 'p3p':
            result = block_data.estimate_pose_p3p(idx)
            if result:
                R, t, error = result
                display_pose_estimation_result(R, t, error=error)
        elif method.startswith('ransac'):
            ransac_method = 'dlt' if method.endswith('dlt') else 'p3p'
            result = block_data.estimate_pose_ransac(idx, ransac_method)
            if result:
                R, t, inliers, error = result
                display_pose_estimation_result(R, t, inliers, error)
                
    except ValueError:
        print("请输入有效的数字")
    except Exception as e:
        print(f"处理过程中出错: {str(e)}")

def main():
    """主程序入口"""
    # 设置中文支持
    if sys.platform == 'win32':
        locale.setlocale(locale.LC_ALL, 'Chinese_China.UTF-8')
    else:
        locale.setlocale(locale.LC_ALL, 'zh_CN.UTF-8')

    # 创建Block实例
    block_data = Block()

    # 从XML文件加载数据
    xml_path = r"E:\slam_work\work_1_high.xml"  # 可以根据需要修改路径
    if not os.path.exists(xml_path):
        print(f"错误：找不到XML文件: {xml_path}")
        return -1

    if not block_data.load_from_xml(xml_path):
        print("无法加载XML文件")
        return -1

    while True:
        show_menu()
        try:
            choice = int(input())
            
            if choice == 0:
                print("程序退出")
                break
            elif choice == 1:
                display_photo_info(block_data.camera.photos)
                display_tiepoint_info(block_data.tiepoints)
            elif choice == 2:
                process_pose_estimation(block_data, 'dlt')
            elif choice == 3:
                process_pose_estimation(block_data, 'p3p')
            elif choice == 4:
                process_pose_estimation(block_data, 'ransac_dlt')
            elif choice == 5:
                process_pose_estimation(block_data, 'ransac_p3p')
            else:
                print("无效的选择，请重试")
                
        except ValueError:
            print("请输入有效的数字")
        except KeyboardInterrupt:
            print("\n程序被用户中断")
            break
        except Exception as e:
            print(f"发生错误: {str(e)}")
            break

if __name__ == "__main__":
    main() 