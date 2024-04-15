#pragma once

#include <queue>
#include <opencv2/opencv.hpp>

//hikang camera
extern int get_picture;
extern int isconnect;
extern std::queue<cv::Mat> hikangPictureQueue;

//kuka

extern int isconnectkuka;
extern int isrunpath;
extern std::vector<std::vector<double>> path_m;