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
#include "kuka_eki_hw_interface.h"
#include <chrono>
#include <fstream>

namespace device{
    /**
     * @brief 显示机器人选择主窗口
     * 
     * @param p_open 
     * @param log 
     */
    void ShowRobotWindow(bool* p_open, AppLog* log);

    /**
     * @brief 显示选择和配置窗口
     * 
     * @param label 
     * @param log 
     * @return int 
     */
    int ShowRobotSelector(const char* label, AppLog* log);


    kuka::CoordinateType ShowCoordinateSelector(const char* label, AppLog* log);
    kuka::MovingType ShowMovingTypeSelector(const char* label, AppLog* log);
    void runpath(AppLog* log, kuka::EKI* eki);
    void configToolNum(int ToolNum, kuka::EKI* eki, AppLog* log);

    /**
     * @brief 连接机器人
     * 
     * @param ip_address ip address of the Robot
     * @param log App log pointer, AppLog*
     * @param eki 
     * @return  
     */
    void ConnectRobot(const char* ip_address, AppLog* log, kuka::EKI* eki);
}
