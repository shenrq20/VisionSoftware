#include <iostream>
#include "kuka_eki_hw_interface/kuka_eki_hw_interface.h"    // 包含机器人控制接口
#include "genicam.h"    // 包含相机控制接口
#include <cmath>
#include <numbers>
#include "point_cloud.h"    // 包含点云处理头文件

int main(int argc, char **argv) {
    // # 检查输入参数个数
    if (argc != 5) {
        return -1;
    }

    // # 创建并连接相机 
    auto camera = new GENICAM("10.1.0.102");
    camera->Connect();

    // # 创建并连接机器人控制器
    auto eki = new eki::EKI;
    eki->Init("10.1.0.101", "54600");
    eki->ready_ = 1;
    eki->Connect();

    // # 参数配置
    // ## 设置碰压参数
    double OvPro = 30;
    // ## 配置碰压进给量为0
    {
        eki::ControlStruct cs;
        cs.control_type_ = eki::kConfig;
        cs.kv_int_.emplace_back("Advance", 0);
        eki->Write(cs);
    }
    
    // ## 配置碰压进给量为0
    /*{
        eki::ControlStruct cs;
        cs.control_type_ = eki::kConfig;
        cs.kv_int_.emplace_back("BaseNum", 1);
        eki->Write(cs);
    }*/
    
    // ## 配置程序速度
    {
        eki::ControlStruct cs;
        cs.control_type_ = eki::kConfig;
        cs.kv_double_.emplace_back("OvPro", OvPro);
        eki->Write(cs);
    }
    // ## 配置工具号 
    {
        eki::ControlStruct cs;
        cs.control_type_ = eki::kConfig;
        cs.kv_int_.emplace_back("ToolNum", 1);
        eki->Write(cs);
    }

    // # 根据输入参数设置起点
    std::vector<double> start_point;
    if (std::string(argv[1]) == "H") {  
        // ## 水平姿态
        start_point = {std::stod(std::string(argv[2])), std::stod(std::string(argv[3])),
                       std::stod(std::string(argv[4])), 0, 90, 0};  
    } else if (std::string(argv[1]) == "V") {
        // ## 垂直姿态
        start_point = {std::stod(std::string(argv[2])), std::stod(std::string(argv[3])),
                       std::stod(std::string(argv[4])), -86, 0, -90};
    } else if (std::string(argv[1]) == "E") {
        // ## 工件水平焊缝扫描
        // start_point = {std::stod(std::string(argv[2])), std::stod(std::string(argv[3])),
        //                std::stod(std::string(argv[4])), 83, 85, 81};
        start_point = {std::stod(std::string(argv[2])), std::stod(std::string(argv[3])),
                       std::stod(std::string(argv[4])), 83, 85, 81};

    }

     // # 安全位置
     // ## 定义安全位置
    std::vector<double> safe_position = {1880, -100, 700, 90, 90, 90};
    //std::vector<double> safe_position = {2122, -184.86, 600, 90, 90, 90}; 

    // ## PTPRelTool做小范围运动
    std::vector<std::vector<double>> rel;
    //rel.push_back({0, 0, 0, -90, -15, 0});
    rel.push_back({0, 0, 0, 0, 10, 0});
    int relFlag = 0;
    
    // ## 移动到安全位置
    {
        // ### 等待机器人就绪
        while (!eki->IsReady()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        // ### 创建控制结构体 
        eki::ControlStruct control_struct;
        // ### 设置为运动控制模式
        control_struct.control_type_ = eki::kMoving;
        // ### 设置为PTP点到点运动
        control_struct.moving_type_ = eki::kPTP;
        // ### 设置坐标系为笛卡尔坐标
        control_struct.coordinate_type_ = eki::kCartesianCoordinateSystem;
        // ### 设置目标位置为安全位置
        control_struct.target_cartesian_position_ = safe_position;

        // ### 发送控制命令
        eki->Write(control_struct);
        // ### 打印目标位置 
        std::cerr << "Move To Safe Point\n";
        std::cerr << "Set Target: {";
        for (auto it = control_struct.target_cartesian_position_.begin();
             it != control_struct.target_cartesian_position_.end(); ++it) {
            if (std::next(it) != control_struct.target_cartesian_position_.end()) {
                std::cerr << *it << ", ";
            } else {
                std::cerr << *it << "}" << std::endl;
            }
        }

        // ### 等待机器人运动完成
        while (!eki->IsReady()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }


    // # 根据相机采集点云计划轨迹
    // double range = 350;
    // std::vector<std::vector<std::vector<double>>> positions;
    // positions.push_back(posi);
    double step = 10;
    double range = 350;
    std::vector<std::vector<std::vector<double>>> positions;    
    positions = processing(eki, camera, start_point, step, range);
    
    std::cout<<"size of output: "<<positions.size()<<std::endl;


    // # 记录结束位置
    auto end_point = eki->GetPosition();

    // # 打磨过程
    // ## 遍历轨迹进行碰压
    for (auto &&j: positions) {
        // 插入接近点以及离开点
        // std::vector<std::vector<double>>j = positions[0];
        std::vector<double> nearPoint = j[0];
        nearPoint[0] += 20;
        nearPoint[2] += 20;
        std::cout<< nearPoint[0] << " " << nearPoint[1] << " " << nearPoint[2] << std::endl;
        std::vector<double> exitPoint = j[j.size()-1];
        exitPoint[0] -= 20;
        exitPoint[2] += 20;
        std::cout<< exitPoint[0] << " " << exitPoint[1] << " " << exitPoint[2] << std::endl;
        // j.insert(j.begin(), nearPoint);
        // j.insert(j.end(), exitPoint);

        //reverse(j.begin(),j.end()); 
        {
            while (!eki->IsReady()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            // ### 设置碰压进给量为0
            eki::ControlStruct cs;
            cs.control_type_ = eki::kConfig;
            cs.kv_int_.emplace_back("Advance", 1); 
        }

        // ## 配置程序速度
        {
            eki::ControlStruct cs;
            cs.control_type_ = eki::kConfig;
            cs.kv_double_.emplace_back("OvPro", OvPro);
            eki->Write(cs);
        }
        // ### 垂直碰压翻转顺序
        if (std::string(argv[1]) == "V"){
            std::reverse(j.begin(), j.end());
        }


        // ### PTP运动到碰压点
        {
            while (!eki->IsReady()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            // ### PTP运动控制指令
            eki::ControlStruct control_struct;
            control_struct.control_type_ = eki::kMoving;
            control_struct.moving_type_ = eki::kPTP;
            control_struct.coordinate_type_ = eki::kCartesianCoordinateSystem;

            // ### 设置目标位置
            control_struct.target_cartesian_position_ = end_point;
            control_struct.target_cartesian_position_[0] = nearPoint[0];
            control_struct.target_cartesian_position_[2] = nearPoint[2];
            control_struct.target_cartesian_position_[1] = nearPoint[1];

            // ### 设置目标位置为安全位置
            // control_struct.target_cartesian_position_ = nearPoint;


            eki->Write(control_struct);
            // ### 打印目标位置
            std::cerr << "Set Target: {";
            for (auto it = control_struct.target_cartesian_position_.begin();
                    it != control_struct.target_cartesian_position_.end(); ++it) {
                if (std::next(it) != control_struct.target_cartesian_position_.end()) {
                    std::cerr << *it << ", ";
                } else {
                    std::cerr << *it << "}" << std::endl;
                }
            }
            // ### 等待运动完成
            while (!eki->IsReady()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }

        {
            if (relFlag == 0) {
                // ### 循环每个相对运动目标位置
                for(auto &&r : rel) {
                    while (!eki->IsReady()) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    }
                    // ### 创建控制结构体
                    eki::ControlStruct control_struct;
                    // ### 设置为运动控制
                    control_struct.control_type_ = eki::kMoving;
                    // ### 设置为PTPRelTool相对运动
                    control_struct.moving_type_ = eki::kPTPRelTool;
                    // ### 设置坐标系为笛卡尔坐标
                    control_struct.coordinate_type_ = eki::kCartesianCoordinateSystem;
                    // ### 设置相对目标位置 
                    control_struct.target_cartesian_position_ = r;

                    // ### 发送控制命令
                    eki->Write(control_struct);
                    // ### 打印目标位置 PrintTarget(control_struct);
                    std::cerr << "Set Relative Target: {";
                    for (auto it = control_struct.target_cartesian_position_.begin();
                            it != control_struct.target_cartesian_position_.end(); ++it) {
                        if (std::next(it) != control_struct.target_cartesian_position_.end()) {
                            std::cerr << *it << ", ";
                        } else {
                            std::cerr << *it << "}" << std::endl;
                        }
                    }
                    // ### 等待运动完成
                    while (!eki->IsReady()) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    }
                }
            }
        }
        
        end_point = eki->GetPosition();

        // ##设置进给量启动碰压  
        {
            // ### 等待机器人就绪
            while (!eki->IsReady()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            eki::ControlStruct cs;
            cs.control_type_ = eki::kConfig;
            cs.kv_int_.emplace_back("Advance", 5);
            // ### 发送控制命令
            eki->Write(cs);
        }

        // ## 设置碰压参数
        {
            eki::ControlStruct cs;
            cs.control_type_ = eki::kConfig;
            cs.kv_double_.emplace_back("OvPro", OvPro);
            cs.kv_double_.emplace_back("VelCP", 0.025 * (100.0 / OvPro));
            eki->Write(cs);
        }
    /*
        // ## 启动力控制
        {
            eki::ControlStruct cs;
            cs.control_type_ = eki::kConfig;
            cs.kv_int_.emplace_back("ForceControl", 1);
            cs.kv_int_.emplace_back("MainAxis", 1);
            // ### 发送控制命令
            eki->Write(cs);
        }
        */

        // 按路径循环打磨
        for (int t = 0; t < 100; t++){
            std::cout<<"第"<<t+1<<"遍打磨"<<std::endl;
            if (t>1-1) {
                std::cout<<"按q退出，任意建继续打磨"<<std::endl;
                char exitFlag0;
                std::cin >> exitFlag0;
                if (exitFlag0 == 'q'){
                    break;
                }
            }
            // KPTP 快速运动到第一点
            {
                while (eki->act_advance_ != 0) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                while (!eki->IsReady()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                // ### PTP运动控制指令
                eki::ControlStruct control_struct;
                control_struct.control_type_ = eki::kMoving;
                control_struct.moving_type_ = eki::kPTP;

                // ### 设置目标位置
                control_struct.target_cartesian_position_ = end_point;
                control_struct.target_cartesian_position_[2] = nearPoint[2];
                control_struct.target_cartesian_position_[1] = nearPoint[1];
                control_struct.target_cartesian_position_[0] = nearPoint[0];

                // ### 设置目标位置为安全位置
                // control_struct.target_cartesian_position_ = nearPoint;

                eki->Write(control_struct);
                // ### 打印目标位置
                std::cerr << "Set Target: {";
                for (auto it = control_struct.target_cartesian_position_.begin();
                    it != control_struct.target_cartesian_position_.end(); ++it) {
                    if (std::next(it) != control_struct.target_cartesian_position_.end()) {
                        std::cerr << *it << ", ";
                    } else {
                        std::cerr << *it << "}" << std::endl;
                    }
                }
                // ### 等待运动完成
                while (!eki->IsReady()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            }

            // ## LIN运动碰压
            for (auto &&i: j) {

                // ### 等待机器人就绪
                while (!eki->IsReady()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                // ### 创建控制结构体
                eki::ControlStruct control_struct;
                // ### 设置为运动控制
                control_struct.control_type_ = eki::kMoving;
                control_struct.moving_type_ = eki::kLIN;
                // ### 设置坐标系为笛卡尔坐标
                control_struct.coordinate_type_ = eki::kCartesianCoordinateSystem;
                // ### 设置相对目标位置 
                control_struct.target_cartesian_position_ = end_point;
                control_struct.target_cartesian_position_[0] = i[0];
                control_struct.target_cartesian_position_[2] = i[2];
                control_struct.target_cartesian_position_[1] = i[1];
                
                //control_struct.target_cartesian_position_[0] = j[i][0];
                //control_struct.target_cartesian_position_[2] = j[i][2];
                //control_struct.target_cartesian_position_[1] = j[i][1];
                
                control_struct.target_cartesian_position_.resize(6);
                
    //            control_struct.target_cartesian_position_[0] -= 1;
    //            control_struct.target_cartesian_position_[2] += tcp_fix_z;
                // ### 发送控制命令
                eki->Write(control_struct);
                
                // ### 打印目标位置
                // std::cerr << "Set Target: {";
                // std::cout << j[i][0] << " " << j[i][1] << " " << j[i][2] << "\n";
                
                std::cerr << "Grinding ...";
                std::cerr << "Set Target: {";
                for (auto it = control_struct.target_cartesian_position_.begin();
                    it != control_struct.target_cartesian_position_.end(); ++it) {
                    if (std::next(it) != control_struct.target_cartesian_position_.end()) {
                        std::cerr << *it << ", ";
                    } else {
                        std::cerr << *it << "}" << std::endl;
                    }
                }
                    
                while (!eki->IsReady()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            }

            // KPTP 快速运动到退出
            {
                while (eki->act_advance_ != 0) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                while (!eki->IsReady()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                // ### PTP运动控制指令
                eki::ControlStruct control_struct;
                control_struct.control_type_ = eki::kMoving;
                control_struct.moving_type_ = eki::kPTP;

                // ### 设置目标位置
                control_struct.target_cartesian_position_ = end_point;
                control_struct.target_cartesian_position_[2] = exitPoint[2];
                control_struct.target_cartesian_position_[1] = exitPoint[1];
                control_struct.target_cartesian_position_[0] = exitPoint[0];

                // ### 设置目标位置为安全位置
                // control_struct.target_cartesian_position_ = nearPoint;

                eki->Write(control_struct);
                // ### 打印目标位置
                std::cerr << "Set Target: {";
                for (auto it = control_struct.target_cartesian_position_.begin();
                    it != control_struct.target_cartesian_position_.end(); ++it) {
                    if (std::next(it) != control_struct.target_cartesian_position_.end()) {
                        std::cerr << *it << ", ";
                    } else {
                        std::cerr << *it << "}" << std::endl;
                    }
                }
                // ### 等待运动完成
                while (!eki->IsReady()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            }
        }

        // ### PTP运动到安全点
        {
            while (eki->act_advance_ != 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            while (!eki->IsReady()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            // ### PTP运动控制指令
            eki::ControlStruct control_struct;
            control_struct.control_type_ = eki::kMoving;
            control_struct.moving_type_ = eki::kPTP;

            // ### 设置目标位置
            control_struct.target_cartesian_position_ = safe_position;
            // control_struct.target_cartesian_position_[2] = exitPoint[2];
            // control_struct.target_cartesian_position_[1] = exitPoint[1];
            // control_struct.target_cartesian_position_[1] = exitPoint[0];

            // ### 设置目标位置为安全位置
            // control_struct.target_cartesian_position_ = nearPoint;

            eki->Write(control_struct);
            // ### 打印目标位置
            std::cerr << "Set Target: {";
            for (auto it = control_struct.target_cartesian_position_.begin();
                 it != control_struct.target_cartesian_position_.end(); ++it) {
                if (std::next(it) != control_struct.target_cartesian_position_.end()) {
                    std::cerr << *it << ", ";
                } else {
                    std::cerr << *it << "}" << std::endl;
                }
            }
            // ### 等待运动完成
            while (!eki->IsReady()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        relFlag += 1;
    }
//     // 将输出路径中的所有y都偏移10mm
//     for (auto &&i: j) {
//         i[1] += 10;
//     }
        
//     end_point = eki->GetPosition();

//     // ##设置进给量启动碰压  
//     {
//         // ### 等待机器人就绪
//         while (!eki->IsReady()) {
//             std::this_thread::sleep_for(std::chrono::milliseconds(20));
//         }
//         eki::ControlStruct cs;
//         cs.control_type_ = eki::kConfig;
//         cs.kv_int_.emplace_back("Advance", 0);
//         // ### 发送控制命令
//         eki->Write(cs);
//     }
// 	/*
//     // ## 启动力控制
//     {
//         eki::ControlStruct cs;
//         cs.control_type_ = eki::kConfig;
//         cs.kv_int_.emplace_back("ForceControl", 1);
//         cs.kv_int_.emplace_back("MainAxis", 1);
//         // ### 发送控制命令
//         eki->Write(cs);
//     }
//     */
//     for (int i = 0; i < 2; i++){
//         std::cout<<"第"<<i+1<<"遍打磨"<<std::endl;
//         // ## LIN运动碰压
//         for (auto &&i: j) {
//         // for (auto i = 0; i < j.size(); i++) {
//         //for (auto i = 0; i < 100; i++) {
//             // ### 等待机器人就绪
//             while (!eki->IsReady()) {
//                 std::this_thread::sleep_for(std::chrono::milliseconds(10));
//             }
//             // ### 创建控制结构体
//             eki::ControlStruct control_struct;
//             // ### 设置为运动控制
//             control_struct.control_type_ = eki::kMoving;
//             control_struct.moving_type_ = eki::kLIN;
//             // ### 设置坐标系为笛卡尔坐标
//             control_struct.coordinate_type_ = eki::kCartesianCoordinateSystem;
//             // ### 设置相对目标位置 
//             control_struct.target_cartesian_position_ = end_point;
//             control_struct.target_cartesian_position_[0] = i[0];
//             control_struct.target_cartesian_position_[2] = i[2];
//             control_struct.target_cartesian_position_[1] = i[1];
            
//             //control_struct.target_cartesian_position_[0] = j[i][0];
//             //control_struct.target_cartesian_position_[2] = j[i][2];
//             //control_struct.target_cartesian_position_[1] = j[i][1];
            
//             control_struct.target_cartesian_position_.resize(6);
            
// //            control_struct.target_cartesian_position_[0] -= 1;
// //            control_struct.target_cartesian_position_[2] += tcp_fix_z;
//             // ### 发送控制命令
//             eki->Write(control_struct);
            
//             // ### 打印目标位置
//             // std::cerr << "Set Target: {";
//             // std::cout << j[i][0] << " " << j[i][1] << " " << j[i][2] << "\n";
            
//             std::cerr << "Grinding ...";
//             std::cerr << "Set Target: {";
//             for (auto it = control_struct.target_cartesian_position_.begin();
//                 it != control_struct.target_cartesian_position_.end(); ++it) {
//                 if (std::next(it) != control_struct.target_cartesian_position_.end()) {
//                     std::cerr << *it << ", ";
//                 } else {
//                     std::cerr << *it << "}" << std::endl;
//                 }
//             }
            
                
//             while (!eki->IsReady()) {
//                 std::this_thread::sleep_for(std::chrono::milliseconds(10));
//             }
            
//         }
//     }
    

    
    // grinding processing done

    // # 结束复位模块
    // ## 退出碰压
    {
        while (eki->act_advance_ != 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        eki::ControlStruct cs;
        cs.control_type_ = eki::kConfig;
        cs.kv_double_.emplace_back("OvPro", OvPro);
        cs.kv_double_.emplace_back("Advance", 1);
        // ## 发送控制命令
        eki->Write(cs);
    }

    // ## 停止力控制
    {
        eki::ControlStruct cs;
        cs.control_type_ = eki::kConfig;
        cs.kv_int_.emplace_back("ForceControl", 0);
        cs.kv_int_.emplace_back("MainAxis", 0);
        eki->Write(cs);
    }

    // ## 移动到安全位置
    {
        while (!eki->IsReady()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        eki::ControlStruct control_struct;
        control_struct.control_type_ = eki::kMoving;
        control_struct.moving_type_ = eki::kPTP;
        control_struct.coordinate_type_ = eki::kCartesianCoordinateSystem;
        control_struct.target_cartesian_position_ = safe_position;

        eki->Write(control_struct);
        std::cerr << "Set Target: {";
        for (auto it = control_struct.target_cartesian_position_.begin();
             it != control_struct.target_cartesian_position_.end(); ++it) {
            if (std::next(it) != control_struct.target_cartesian_position_.end()) {
                std::cerr << *it << ", ";
            } else {
                std::cerr << *it << "}" << std::endl;
            }
        }

        while (!eki->IsReady()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    // ## 断开连接
    eki->Disconnect();
    camera->Disconnect();
    return 0;
}

