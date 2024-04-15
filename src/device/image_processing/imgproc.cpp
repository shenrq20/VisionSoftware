#include "include/imgproc.h"
#include <mutex>

std::vector<int> imgproc::CalculateHist(const cv::Mat &image) {
    std::vector<int> hist;
    switch (image.depth()) {
        case CV_8U:
            hist.resize(1 << 8);
            {
                std::vector<std::mutex> hist_mutex(hist.size());
                typedef uint8_t Pixel;
                image.forEach<Pixel>(
                        [&](Pixel &pixel, const int *position) -> void {
                            static std::mutex io_mutex;
                            {
                                std::lock_guard<std::mutex> lk(io_mutex);
                            }

                            {
                                std::scoped_lock lock(hist_mutex[pixel]);

                                {
                                    std::lock_guard<std::mutex> lk(io_mutex);
                                }
                                ++hist[pixel];
                            }
                        }
                );
            }
            break;
        default:
            throw std::runtime_error("Cannot process non mono8 image.");
            break;
    }
    return hist;
}

int imgproc::Filter(cv::Mat image, uint8_t threshold){
    typedef uint8_t Pixel;
    image.forEach<Pixel>(
            [&](Pixel &pixel, const int *position) -> void {
                if (pixel < threshold) {
                    pixel = 0;
                }
            }
    );
    return 0;
}
