SLAM-homework（前四题使用C++语言；第五题有关于DLT&P3P和RANSAC使用python编写，在DLT+P3P文件夹中）

项目结构：

SLAM_3D_Reconstruction/

├── 引用/

├── 外部依赖项/

├── 头文件/

│   ├── geometry/

│   │   ├── camera.hpp       

│   │   ├── essential_matrix.hpp//本质矩阵

│   │   ├── pnp_solver.hpp//DLT+P3P

│   │   └── rigid_transform.hpp//刚体变换参数

│   ├── image/

│   │   └── image_processor.hpp//图像去畸变、坐标畸变正反过程

│   ├── types/

│   │   ├── block.hpp

│   │   ├── common_types.hpp//基础类

│   │   └── slam_types.hpp

│   └── tinyxml2.h//xml文件的读取第三方库实现

├── 源文件/

│   ├── geometry/

│   │   ├── camera.cpp

│   │   ├── essential_matrix.cpp

│   │   ├── pnp_solver.cpp

│   │   └── rigid_transform.cpp

│   ├── image/

│   │   └── image_processor.cpp

│   ├── types/

│   │   ├── block.cpp

│   │   ├── common_types.cpp

│   │   └── slam_types.cpp

│   ├── main.cpp

│   └── tinyxml2.cpp

└── 资源文件/
