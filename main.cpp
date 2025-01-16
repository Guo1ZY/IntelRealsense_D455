#include "Realsense.hpp"
#include <opencv2/opencv.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <sys/time.h>
int main()
{
    Realsense camera;

    // 初始化相机
    camera.init();
    camera.setCamera();
    
    //time
    timeval tt1, tt2;
        // 创建点云可视化
        pcl::visualization::CloudViewer viewer("Cloud Viewer");

    while (!viewer.wasStopped())
    {
        gettimeofday(&tt1, NULL);
        // 获取彩色图像
        auto frames = camera.getImg();
        if (!frames.empty())
        {
            cv::imshow("Color Image", frames[0]);
        }

        // 获取对齐的点云和图像
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        cv::Mat colorImage;
        // camera.getCloud(cloud, colorImage);
        gettimeofday(&tt2, NULL);
        //timeuse in ms
        long timeuse = 1000 * (tt2.tv_sec - tt1.tv_sec) + (tt2.tv_usec - tt1.tv_usec)*1.0/1000;
        std::cout << "timeuse: " << timeuse << "ms" << std::endl;
        // 显示点云
        // viewer.showCloud(cloud);

        if (cv::waitKey(1) == 27)
        { // 按 ESC 退出
            break;
        }
    }

    // 关闭相机
    camera.close();
    return 0;
}
