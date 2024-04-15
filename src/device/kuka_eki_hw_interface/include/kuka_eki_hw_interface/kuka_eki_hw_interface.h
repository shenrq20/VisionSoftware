/**
 * @file kuka_eki_hw_interface.h
 * @author Shibo LIU (shibo.liu@saiwider.com)
 * @brief 设备-Kuka机器人类头文件
 * @version 0.1
 * @date 2023-12-26
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <utility>
#include <vector>
#include <string>

#include <boost/asio.hpp>
// #include <iostream>
#include <fstream>
#include <mutex>
#include <chrono>
#include <thread>

// Nkuka: namespace kuka
namespace kuka {
    enum ControlType {
        kConfig,
        kMoving,
    };
    enum MovingType {
        kLIN,
        kPTP,
        kLINRelTool,
        kPTPRelTool,
    };
    enum CoordinateType {
        kCartesianCoordinateSystem,
        kAxisCoordinateSystem,
    };

    template<typename T>
    class KeyValue{
    public:
        std::string key_;
        T value_;
        KeyValue(std::string  key, T value);
        ~KeyValue();
    };

    template<typename T>
    KeyValue<T>::KeyValue(std::string  key, T value) : key_(std::move(key)), value_(value) {
    }

    template<typename T>
    KeyValue<T>::~KeyValue() = default;

    class ControlStruct {
    public:
        ControlType control_type_ = kConfig;
        std::vector<KeyValue<int>>kv_int_;
        std::vector<KeyValue<double>> kv_double_;
        MovingType moving_type_;
        CoordinateType coordinate_type_;
        std::vector<double> target_cartesian_position_;
        std::vector<double> target_axis_position_;
    };

    class EKI {
    public:
        int moving_status_{};
        int pro_move_{};
        int ready_{};
        std::mutex ready_mutex_, position_mutex_;
        bool connected_ = false;
        int advance_ = 3, act_advance_ = 3;
        int command_size_{};

        EKI();

        ~EKI();

        void Init(const std::string &ip_address, const std::string &port);

        void Connect();

        void Disconnect();

        void Read();

        std::vector<double> GetPosition(CoordinateType coordinate_type = CoordinateType::kCartesianCoordinateSystem);

        void Write(const ControlStruct & control_struct);

        void SetReady(int i);

        int IsReady();

        int (*Monitor)(EKI *eki) = &DefaultMonitor;

        int PrintStatus();

        int LogStatus(std::ofstream &fs);

        /**
         * @brief 控制机械臂移动
         * 
         * @param eki kuka机器人类
         * @param targetPoint 目标位置
         * @param controlType 控制类型，包含kConfig,kMoving
         * @param movingType 运动类型，包含kLIN, kPTP, kLINRelTool, kPTPRelTool
         * @param speed 机器人运动速度 m/s
         * @param coordinateType 坐标类型，包含kCartesianCoordinateSystem, kAxisCoordinateSystem
         * @param is_print_info 是否打印信息
         * @return int 无错误范围0，有错误返回-1
         */
        int Move(EKI* eki, std::vector<double> targetPoint, 
                kuka::ControlType controlType,
                kuka::MovingType movingType,
                double speed,
                kuka::CoordinateType coordinateType,
                bool is_print_info);


    private:
        const unsigned int n_dof_ = 6;
        std::vector<double> cartesian_position_;
        std::vector<double> axis_position_;
        std::vector<double> joint_velocity_;
        std::vector<double> joint_effort_;

        std::vector<std::string> cartesian_coordinate_name_ = {"X", "Y", "Z", "A", "B", "C"};
        std::vector<std::string> axis_coordinate_name_ = {"A1", "A2", "A3", "A4", "A5", "A6"};

        std::thread monitor_thread_;

        std::chrono::time_point<std::chrono::steady_clock> prev_, now_;

        // EKI
        std::string eki_server_address_;
        std::string eki_server_port_;
        //int eki_cmd_buff_len_{};

        // EKI socket Read/Write
        int eki_read_state_timeout_ = 60;  // [s]; settable by parameter (default = 5)
        boost::asio::io_service ios_;
        boost::asio::deadline_timer deadline_;

        boost::asio::ip::udp::socket eki_server_socket_;
        boost::asio::ip::udp::endpoint eki_server_endpoint_;

        void eki_check_read_state_deadline();

        static void eki_handle_receive(const boost::system::error_code &ec, size_t length,
                                       boost::system::error_code *out_ec, size_t *out_length);

        bool eki_read_state();

        bool eki_write_command(const ControlStruct & control_struct);

        static int DefaultMonitor(EKI* eki);
    };

} // namespace eki
