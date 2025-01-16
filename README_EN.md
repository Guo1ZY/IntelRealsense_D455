* # Intel Realsense D455 Camera Library

  This library provides an interface for the Intel RealSense D455 camera, supporting camera initialization, capturing color images, and processing point cloud data. The library integrates with the RealSense SDK, OpenCV, and PCL.

  ## Features

  - **Initialization**: Configure the camera with default or custom settings.
  - **Color Image Capture**: Retrieve color frames from the camera.
  - **Point Cloud Generation**: Generate aligned point cloud data for visualization and processing.
  - **Camera Configuration**: Adjust camera parameters such as exposure and gain.

  ## Requirements

  - **C++ Compiler**: Supports C++17 standard.
  - **Dependencies**:
    - [Intel RealSense SDK](https://github.com/IntelRealSense/librealsense)
    - [OpenCV](https://opencv.org/)
    - [Point Cloud Library (PCL)](https://pointclouds.org/)

  ## Installation

  1. **Clone the Repository**:

     ```
     git clone https://github.com/Guo1ZY/IntelRealsense_D455
     cd IntelRealsense_D455
     ```

  2. **Build the Project**:

     ```
     mkdir build && cd build
     cmake ..
     make
     ```

  3. **Run the Application**:

     ```
     ./IntelRealsense_D455
     ```

  ## File Structure

  - `main.cpp`: Entry point of the application. Demonstrates camera initialization and data retrieval.
  - `Realsense.hpp` and `Realsense.cpp`: Define and implement the `Realsense` class.
  - `CMakeLists.txt`: Build configuration file for CMake.

  ## Usage

  ### Basic Workflow

  1. **Initialize the Camera**:

     ```
     Realsense camera;
     camera.init();
     camera.setCamera();
     ```

  2. **Capture Color Frames**:

     ```
     auto frames = camera.getImg();
     if (!frames.empty()) {
         cv::imshow("Color Image", frames[0]);
     }
     ```

  3. **Generate Point Cloud**:

     ```
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
     cv::Mat colorImage;
     camera.getCloud(cloud, colorImage);
     ```

  4. **Close the Camera**:

     ```
     camera.close();
     ```

  ### Example

  The following code snippet demonstrates how to capture images and point clouds in the main loop:

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
  
          if (cv::waitKey(1) == 27) { // Press ESC to exit
              break;
          }
      }
  
      camera.close();
      return 0;
  }
  ```

  ## Customization

  You can modify camera parameters such as exposure, gain, and resolution in the `setCamera` method in `Realsense.cpp`.

  ## License

  This project is licensed under the MIT License. See the LICENSE file for details.

  ## Acknowledgments

  - Intel RealSense SDK
  - OpenCV
  - Point Cloud Library
