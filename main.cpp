#include <iostream>
#include <vector>
#include "kuka_eki_hw_interface.h"
// #include "imgproc.h"
// #include "opencv2/imgcodecs.hpp"

int main() {
    std::cout << "1" << std::endl;

    // 测试设备KUKA机器人部分 ------------------------------------------------------------------
    Nkuka::EKI* eki = new Nkuka::EKI;
    eki->Init("10.1.0.101", "54600");
    eki->Connect();
    // 测试KUKA机器人Move方法
    std::vector<double> safe_position = {1880, -100, 700, 90, 90, 90};
    bool isPrintInfo = true;
    eki->Move(eki, safe_position,
             Nkuka::kMoving, Nkuka::kPTP, 
             0.0001,
             Nkuka::kCartesianCoordinateSystem, isPrintInfo);
    
    eki->Disconnect();
    std::cout << "KUKA Disconnected." << std::endl;
    // 释放
    delete eki;
    // ---------------------------------------------------------------------------------------

    // // 测试处理函数中图像处理部分 ------------------------------------------------------------
    // cv::Mat testImage = cv::imread("../image/logo/logo.png");
    // // test-CalculateHist
    // imgproc::ImageProcessor* img_test = new imgproc::ImageProcessor(testImage);
    // // std::vector<int> test_vec = img_test->CalculateHist();
    // img_test->VisualizeHist();
    // // -------------------------------------------------------------------------------------


}

