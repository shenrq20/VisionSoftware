/**
 * @file devConnect.h
 * @author Shibo Liu (shibo.liu@saiwider.com)
 * @brief 设备窗口文档头文件
 * @version 0.1
 * @date 2024-01-18
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#pragma once

#include "log.h"
#include "genicam.h"

namespace device{


    /**
     * @brief Show Camera Selection main window
     * 
     * @param p_open Window open controller
     * @param log Window for log information record 
     */
    void ShowCameraWindow(bool* p_open, AppLog* log);

    /**
     * @brief 
     * 
     * @param label Combo information
     * @param log Window for log information record 
     * @return int Camera type index
     */
    int ShowCameraSelector(const char* label, AppLog* log);
    /**
     * @brief 连接相机Hikang
     * 
     * @param ip_address ip address of the Camera
     * @param log App log pointer, AppLog*
     * @return  
     */
    void ConnectCam(const char* ip_address, AppLog* log);

    void getHikangPicture(GENICAM * camera, AppLog* log);
}
