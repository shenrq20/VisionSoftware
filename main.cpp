#include <iostream>
#include "kuka_eki_hw_interface.h"
#include "imgproc.h"
#include "opencv2/imgcodecs.hpp"

int main() {
    std::cout << "1" << std::endl;

    // 测试设备KUKA机器人部分
    device::EKI* eki = new device::EKI;

    // 测试处理函数中图像处理部分
    cv::Mat testImage = cv::imread("../image/logo/logo.png");
    // test-CalculateHist
    imgproc::ImageProcessor* img_test = new imgproc::ImageProcessor(testImage);
    // std::vector<int> test_vec = img_test->CalculateHist();
    img_test->VisualizeHist();


}

