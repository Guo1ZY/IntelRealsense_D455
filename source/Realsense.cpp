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
#include "Realsense.hpp"
#include <iostream>

/**
 * @brief 构造函数
 * 
 */
Realsense::Realsense() : isInitialized(false), depthScale(0.001f) {}

/**
 * @brief 析构函数
 * 
 */

Realsense::~Realsense()
{
    close();
}

/**
 * @brief 初始化相机
 * 
 */

void Realsense::init()
{
    try
    {
        int resolution_width = 1280; // 默认分辨率宽度
        int resolution_height = 720; // 默认分辨率高度
        int fps = 30;                // 默认帧率
        float min_distance = 0.1f;
        float max_distance = 10.0f; // 默认最大深度距离

        config.enable_stream(RS2_STREAM_COLOR, resolution_width, resolution_height, RS2_FORMAT_BGR8, fps);        // 配置彩色图像
        config.enable_stream(RS2_STREAM_DEPTH, resolution_width / 2, resolution_height / 2, RS2_FORMAT_Z16, fps); // 配置深度图像，分辨率通常较低
        pipeline.start(config);
        isInitialized = true;

        auto device = pipeline.get_active_profile().get_device();
        auto sensor = device.first<rs2::depth_sensor>(); // 获取深度传感器
        depthScale = sensor.get_depth_scale();           // 获取深度缩放

        // 设置深度范围
        // sensor.set_option(RS2_OPTION_MIN_DISTANCE, min_distance);
        // sensor.set_option(RS2_OPTION_MAX_DISTANCE, max_distance);

        // 获取设备信息
        std::string device_name = device.get_info(RS2_CAMERA_INFO_NAME);
        std::string serial_number = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

        // 输出初始化成功信息
        std::cout << "Intel RealSense: " << device_name << " Initialize success!\n";
        std::cout << "SN: " << serial_number << "\n";
        std::cout << "Resolution: " << resolution_width << "x" << resolution_height << " @ " << fps << " FPS\n";
        std::cout << "Depth Range: " << min_distance << "m to " << max_distance << "m\n";
    }
    catch (const rs2::error &e)
    {
        std::cerr << "RealSense error: " << e.what() << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Standard exception: " << e.what() << std::endl;
    }
}

void Realsense::setCamera()
{
    // 设置相机参数，默认其实可以，其他要设置的去官网查
    auto sensor = pipeline.get_active_profile().get_device().first<rs2::depth_sensor>();// 获取深度传感器
    sensor.set_option(RS2_OPTION_GAIN, 16);// 增益
    sensor.set_option(RS2_OPTION_EXPOSURE, 5000);// 曝光
    sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);// 自动曝光
    // sensor.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, 0);// 自动白平衡
    // sensor.set_option(RS2_OPTION_WHITE_BALANCE, 4600);// 白平衡
    // sensor.set_option(RS2_OPTION_BRIGHTNESS, 0);// 亮度
    // sensor.set_option(RS2_OPTION_CONTRAST, 50);// 对比度
    // sensor.set_option(RS2_OPTION_GAMMA, 300);// 伽马
    // sensor.set_option(RS2_OPTION_HUE, 0);// 色调
    // sensor.set_option(RS2_OPTION_SATURATION, 64);// 饱和度
    // sensor.set_option(RS2_OPTION_SHARPNESS, 50);// 锐度
    // sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_PRIORITY, 0);// 自动曝光优先级
    // sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_LIMIT, 1000);// 自动曝光限制
    // sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_MODE, 1);// 自动曝光模式
    // sensor.set_option(RS2_OPTION_LASER_POWER, 150);// 激光功率
    // sensor.set_option(RS2_OPTION_ACCURACY, 3);// 精度
    // sensor.set_option(RS2_OPTION_MOTION_RANGE, 0);// 运动范围
    // sensor.set_option(RS2_OPTION_FILTER_OPTION, 0);// 滤波器选项
    // sensor.set_option(RS2_OPTION_CONFIDENCE_THRESHOLD, 1);// 置信度阈值
    // sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1);// 发射器启用
    // sensor.set_option(RS2_OPTION_FRAMES_QUEUE_SIZE, 1);// 帧队列大小
    // sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 0);// 内部相机同步模式
}

void Realsense::close()
{
    if (isInitialized)
    {
        pipeline.stop();
        isInitialized = false;
    }
    // 输出关闭成功信息
    std::cout << "Camera close success!\n"<<std::endl;
}

std::vector<cv::Mat> Realsense::getImg()
{
    std::vector<cv::Mat> frames;
    if (!isInitialized)
    {
        std::cerr << "Realsense not initialized!" << std::endl;
        return frames;
    }

    try
    {
        frameset = pipeline.wait_for_frames();
        auto color = frameset.get_color_frame();

        if (color)
        {
            cv::Mat colorMat(cv::Size(color.get_width(), color.get_height()),
                             CV_8UC3, (void *)color.get_data(), cv::Mat::AUTO_STEP);
            frames.push_back(colorMat.clone());
        }
    }
    catch (const rs2::error &e)
    {
        std::cerr << "RealSense error: " << e.what() << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Standard exception: " << e.what() << std::endl;
    }

    return frames;
}

void Realsense::getCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr vis_cloud_return, cv::Mat &image_return)
{
    if (!isInitialized)
    {
        std::cerr << "Realsense not initialized!" << std::endl;
        return;
    }

    try
    {
        frameset = pipeline.wait_for_frames();
        auto color = frameset.get_color_frame();
        auto depth = frameset.get_depth_frame();

        const int width = depth.get_width();
        const int height = depth.get_height();

        vis_cloud_return->clear();

        cv::Mat colorMat(cv::Size(color.get_width(), color.get_height()),
                         CV_8UC3, (void *)color.get_data(), cv::Mat::AUTO_STEP);
        image_return = colorMat.clone();

        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                float depthValue = depth.get_distance(x, y);
                if (depthValue > 0 && depthValue < 5)
                { // Filter invalid and far points
                    pcl::PointXYZRGB point;
                    point.z = -depthValue;
                    point.x = -(x - width / 2) * depthValue * depthScale;
                    point.y = -(y - height / 2) * depthValue * depthScale;

                    auto pixel = colorMat.at<cv::Vec3b>(y, x);
                    point.r = pixel[2];
                    point.g = pixel[1];
                    point.b = pixel[0];

                    vis_cloud_return->points.push_back(point);
                }
            }
        }

        vis_cloud_return->width = static_cast<uint32_t>(vis_cloud_return->points.size());
        vis_cloud_return->height = 1;
        vis_cloud_return->is_dense = true;
    }
    catch (const rs2::error &e)
    {
        std::cerr << "RealSense error: " << e.what() << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Standard exception: " << e.what() << std::endl;
    }
}
