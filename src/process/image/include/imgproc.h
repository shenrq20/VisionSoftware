/**
 * @file imgproc.h
 * @author Shibo Liu
 * @brief 图像处理相关函数
 * @version 0.1
 * @date 2023-12-04
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

namespace imgproc {

    // 图像处理类的声明
    class ImageProcessor {
    public:
        // 构造函数，接受一个图像作为参数
        ImageProcessor(const cv::Mat &image);

        /**
         * @brief 计算输入图像的直方图
         * @return 包含直方图的 std::vector<int>
         */
        std::vector<int> CalculateHist();

        /**
         * @brief 对图像进行二值化处理
         * @param threshold 二值化阈值
         * @return 0 表示成功，其他值表示错误
         */
        int Filter(uint8_t threshold);

        /**
        * @brief 可视化直方图
        */
        void VisualizeHist();

    private:
        cv::Mat image_;  // 存储输入图像的成员变量
    };

}  // namespace imgproc

