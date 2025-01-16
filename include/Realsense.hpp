/**
 * @file Realsense.cpp
 * @author Guo1ZY(1352872047@qq.com)
 * @brief 黑奴写的Realsense D455相机
 * @version 0.1
 * @date 2025-01-15
 *
 * @copyright Copyright (c) 2025
 *
 */
#ifndef REALSENSE_HPP
#define REALSENSE_HPP

#include <librealsense2/rs.hpp> // Intel RealSense SDK
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <string>
#include <vector>

class Realsense
{
public:
    Realsense();
    ~Realsense();
    /**
     * @brief 初始化相机
     * 
     */
    void init();  
    /**
     * @brief 设置相机参数
     * 
     * @return void
     */
    void setCamera(); 
    /**
     * @brief 关闭相机
     * 
     * @return void
     */
    void close();     

    /**
     * @brief 获取彩色图像
     * 
     * @return std::vector<cv::Mat> 彩色图像
     */
    std::vector<cv::Mat> getImg();      
    /**
     * @brief 获取对齐点云
     * 
     * @param vis_cloud_return 对齐点云
     * @param image_return 对齐图像
     * @return void
     */
    void getCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr vis_cloud_return, cv::Mat &image_return); // 获取对齐点云

private:
    rs2::pipeline pipeline;// 管道
    rs2::config config;// 配置
    rs2::frameset frameset;// 帧集
    rs2::colorizer colorizer;// 着色器

    bool isInitialized;// 是否初始化
    float depthScale;// 深度图像缩放
};

#endif // REALSENSE_HPP