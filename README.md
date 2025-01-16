* # Intel Realsense D455 相机库

* 

* author :Guo1ZY

* Date:2024/1/16

* Version:1.0

  本库提供一个为 Intel RealSense D455 相机设计的接口，支持相机初始化、采集彩色图像和处理点云数据等功能。该库集成了 RealSense SDK，OpenCV 和 PCL。

  ## 功能

  - **初始化**：使用默认或自定义设置配置相机。
  - **彩色图像采集**：获取相机输出的彩色帧。
  - **点云生成**：生成对齐的点云数据，用于可视化和处理。
  - **相机参数配置**：调整曝光、增益等相机参数。

  ## 环境要求

  - **C++ 编译器**：支持 C++17 标准。
  - **依赖性**：
    - [Intel RealSense SDK](https://github.com/IntelRealSense/librealsense)
    - [OpenCV](https://opencv.org/)
    - [Point Cloud Library (PCL)](https://pointclouds.org/)

  ## 安装

  1. **克隆仓库**：

     ```
     git clone https://github.com/Guo1ZY/IntelRealsense_D455
     cd IntelRealsense_D455
     ```

  2. **构建项目**：

     ```
     mkdir build && cd build
     cmake ..
     make
     ```

  3. **运行应用**：

     ```
     ./IntelRealsense_D455
     ```

  ## 文件结构

  - `main.cpp`：应用的入口。示范如何初始化相机和获取数据。
  
  - `Realsense.hpp` 和 `Realsense.cpp`：定义并实现 `Realsense`类。

  - `CMakeLists.txt`：CMake 的构建配置文件。

    ├── CMakeLists.txt
    ├── include
    │   └── Realsense.hpp
    ├── LICENSE
    ├── main.cpp
    ├── README_EN.md
    ├── README.md
    ├── script
    │   ├── main2.py
    │   ├── main.py
    │   ├── realsense.py
    │   └── settings.py
    └── source
        └── Realsense.cpp
  
  ## 使用方法
  
  ### 基础流程

  1. **初始化相机**：

     ```
     Realsense camera;
     camera.init();
     camera.setCamera();
     ```

  2. **采集彩色帧**：

     ```
     auto frames = camera.getImg();
     if (!frames.empty()) {
         cv::imshow("Color Image", frames[0]);
     }
     ```
  
  3. **生成点云**：
  
     ```
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
     cv::Mat colorImage;
     camera.getCloud(cloud, colorImage);
     ```
  
  4. **关闭相机**：
  
     ```
     camera.close();
     ```
  
  ### 示例
  
  下面的代码段显示如何在主循环中采集图像和点云：
  
  ```
  #include "Realsense.hpp"
  #include <opencv2/opencv.hpp>
  #include <pcl/visualization/cloud_viewer.h>
  
  int main() {
      Realsense camera;
      camera.init();
      camera.setCamera();
  
      pcl::visualization::CloudViewer viewer("Cloud Viewer");
  
      while (!viewer.wasStopped()) {
          auto frames = camera.getImg();
          if (!frames.empty()) {
              cv::imshow("Color Image", frames[0]);
          }
  
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
          cv::Mat colorImage;
          camera.getCloud(cloud, colorImage);
          viewer.showCloud(cloud);
  
          if (cv::waitKey(1) == 27) { // 按 ESC 退出
              break;
          }
      }
  
      camera.close();
      return 0;
  }
  ```
  
  ## 自定义
  
  您可以在 `Realsense.cpp` 中的 `setCamera` 方法修改曝光、增益和分辨率等相机参数。
  
  ## 许可证
  
  本项目按 MIT 许可证发布。请参阅 LICENSE 文件以获取详情。
  
  ## 致谢
  
  - Intel RealSense SDK
  - OpenCV
  - Point Cloud Library
