#include "include/imgproc.h"

namespace imgproc {

// 构造函数，接受一个图像作为参数
ImageProcessor::ImageProcessor(const cv::Mat &image) : image_(image) {}

/**
 * @brief 计算输入图像的直方图
 * @return 包含直方图的 std::vector<int>
 */
std::vector<int> ImageProcessor::CalculateHist() {
    std::vector<int> hist;  // 存储直方图的容器
    switch (image_.depth()) {
        case CV_8U:  // 处理图像的像素深度为 8 位的情况
            hist.resize(1 << 8);  // 直方图大小为 256（2^8）个 bin
            {
                std::vector<std::mutex> hist_mutex(hist.size());  // 为每个 bin 创建一个互斥锁
                typedef uint8_t Pixel;  // 图像像素类型为 uint8_t
                image_.forEach<Pixel>(
                    [&](Pixel &pixel, const int *position) -> void {
                        static std::mutex io_mutex;  // 用于控制标准输出的互斥锁
                        {
                            std::lock_guard<std::mutex> lk(io_mutex);  // 加锁保护标准输出
                        }

                        {
                            std::scoped_lock lock(hist_mutex[pixel]);  // 加锁保护当前像素对应的直方图 bin
                            {
                                std::lock_guard<std::mutex> lk(io_mutex);  // 加锁保护标准输出
                            }
                            ++hist[pixel];  // 更新直方图 bin 的计数
                        }
                    }
                );
            }
            break;
        default:
            throw std::runtime_error("Cannot process non mono8 image.");  // 无法处理非单通道 8 位深度的图像
            break;
    }
    return hist;  // 返回计算得到的直方图
}

/**
 * @brief 对图像进行二值化处理
 * @param threshold 二值化阈值
 * @return 0 表示成功，其他值表示错误
 */
int ImageProcessor::Filter(uint8_t threshold) {
    typedef uint8_t Pixel;  // 图像像素类型为 uint8_t
    image_.forEach<Pixel>(
        [&](Pixel &pixel, const int *position) -> void {
            if (pixel < threshold) {
                pixel = 0;  // 小于阈值的像素置为 0
            }
        }
    );
    return 0;  // 成功返回 0
}

/**
 * @brief 可视化直方图
 */
void ImageProcessor::VisualizeHist() {
    std::vector<int> hist = CalculateHist();
    
    // Plotting the histogram using OpenCV
    cv::Mat hist_image(256, 256, CV_8U, cv::Scalar(255));
    int hist_width = 256;
    int hist_height = 256;

    int bin_width = cvRound((double)hist_width / hist.size());

    for (size_t i = 0; i < hist.size(); i++) {
        rectangle(hist_image, cv::Point(i * bin_width, hist_height),
                  cv::Point((i + 1) * bin_width, hist_height - hist[i]),
                  cv::Scalar(0), -1, 8, 0);
    }

    cv::imwrite("Histogram.png", hist_image);
}

}  // namespace imgproc
