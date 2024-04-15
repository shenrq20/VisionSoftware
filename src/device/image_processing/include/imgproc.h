#ifndef INC_IMGPROC_TEST_H
#define INC_IMGPROC_TEST_H

#include <opencv2/opencv.hpp>

namespace imgproc {
    std::vector<int> CalculateHist(const cv::Mat& img);
    int Filter(cv::Mat image, uint8_t threshold);
}

#endif //INC_IMGPROC_TEST_H
