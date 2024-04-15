/**
 * @file devRobConnect.cpp
 * @author Shibo LIU (shibo.liu@saiwider.com)
 * @brief Robot connection section
 * @version 0.1
 * @date 2024-01-22
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <array>
#include <iostream>
#include "imgui.h"
#include "devRobConnect.h"
#include "kuka_eki_hw_interface.h"
#include "log.h"
#include "gvalue.h"

// bool ImGui::ShowStyleSelector(const char* label)
int device::ShowRobotSelector(const char* label, AppLog* log)
{
    static int device_idx = -1;
    if (ImGui::Combo(label, &device_idx, "KUKA\0ABB\0"))
    {
        switch (device_idx)
        {
        case 0: log->AddLog("[info] Configure KUKA Camera\n"); break; // Call configure function
        case 1: log->AddLog("[info] Configure ABB Camera\n"); break;   // Call configure function
        }
    }
    return device_idx;
}

kuka::CoordinateType device::ShowCoordinateSelector(const char* label, AppLog* log)
{
    static int device_idx = -1;
    if (ImGui::Combo(label, &device_idx, "cartesion\0axis\0"))
        {
            switch (device_idx)
            {
            case 0: log->AddLog("[info] choose cartesion position\n"); break; // Call configure function
            case 1: log->AddLog("[info] choose axis position\n"); break;   // Call configure function
            }
        }
    if (device_idx == 0)
        return kuka::kCartesianCoordinateSystem;
    if (device_idx == 1)
        return kuka::kAxisCoordinateSystem;
    return kuka::kCartesianCoordinateSystem;
}

kuka::MovingType device::ShowMovingTypeSelector(const char* label, AppLog* log)
{
    static int device_idx = -1;
    if (ImGui::Combo(label, &device_idx, "kLIN\0kPTP\0kLINRelTool\0kPTPRelTool\0"))
        {
            switch (device_idx)
            {
            case 0: log->AddLog("[info] choose kLIN MovingType\n"); break; 
            case 1: log->AddLog("[info] choose kPTP MovingType\n"); break;   
            case 2: log->AddLog("[info] choose kLINRelTool MovingType\n"); break; 
            case 3: log->AddLog("[info] choose kPTPRelTool MovingType\n"); break;   
            }
        }
    if (device_idx == 0)
        return kuka::kLIN;
    if (device_idx == 1)
        return kuka::kPTP;
    if (device_idx == 2)
        return kuka::kLINRelTool;
    if (device_idx == 3)
        return kuka::kPTPRelTool;
    return kuka::kPTP;
}

void runpathAsync(AppLog* log, kuka::EKI* eki)
{
    std::cout<<"是否连接 "<<isconnectkuka<<std::endl;
    device::runpath(log, eki);
}

void device::ConnectRobot(const char* ip_address, AppLog* log, kuka::EKI* eki)
{
    try{
        // kuka::EKI* eki = new kuka::EKI;
        //eki->Init(ip_address, "54600");
        eki->Init("172.31.1.147", "54600");
        eki->ready_ = 1;
        eki->Connect();
        log->AddLog("[info] Robot connected.\n");
        isconnectkuka = 1;
        // 同时创建一个线程用于机器人运动
        std::thread runpathThread(runpathAsync, log, eki);
        runpathThread.detach();  
        configToolNum(6, eki, log);
    }catch(...){

    log->AddLog("[error] Robot not connected.\n");
    }


    
}

void seendMoveTarget(kuka::EKI* eki, AppLog* log, std::vector<double> target, kuka::MovingType MT,
                    kuka::CoordinateType CT){
    
    std::cout<<"发送轨迹点："<<target[0]<<" "
                            <<target[1]<<" "
                            <<target[2]<<" "
                            <<target[3]<<" "
                            <<target[4]<<" "
                            <<target[5]<<std::endl;
    
    try{
        // ### 创建控制结构体 
        kuka::ControlStruct control_struct;
        // ### 设置为运动控制模式
        control_struct.control_type_ = kuka::kMoving;
        // ### 设置为PTP点到点运动
        control_struct.moving_type_ = MT;
        // ### 设置坐标系为笛卡尔坐标
        control_struct.coordinate_type_ = CT;
        // ### 设置目标位置为安全位置
        control_struct.target_cartesian_position_ = target;

        // ### 发送控制命令
        eki->Write(control_struct);
        log->AddLog("[info] Target position: [%.2f] [%.2f] [%.2f] [%.2f] [%.2f] [%.2f]\n",
                        target[0],target[1],target[2],target[3],target[4],target[5]);
    }catch(...){
    log->AddLog("[error] move fail, check robot connect.\n");
    }
}

void device::runpath(AppLog* log, kuka::EKI* eki)
{
    std::cout<<"是否连接 "<<isconnectkuka<<std::endl;
    std::cout<<"是否连接eki "<<eki->IsReady()<<std::endl;
    while(isconnectkuka > 0){
        try{
            if(isrunpath >= 1){
                if(path_m[0][0] == 0){
                    log->AddLog("[%s] target position error.\n",
                        "error");
                }else{
                    std::cout<<"轨迹点数： "<<path_m.size()<<std::endl;
                    for(int i = 0; i < path_m.size(); i++){


                        kuka::MovingType moving_type_ = kuka::kPTP;
                        kuka::CoordinateType coordinate_type_ = kuka::kCartesianCoordinateSystem;

                        seendMoveTarget(eki, log, path_m[i], moving_type_, coordinate_type_);

                        // ### 等待机器人运动完成
                        while (!eki->IsReady()) {
                            std::this_thread::sleep_for(std::chrono::milliseconds(10));
                        }
            
                    }
                }
                isrunpath--;
                std::cout<<isrunpath<<std::endl;
            }
        }catch(...){

            log->AddLog("[error] Robot not connected.\n");
        }
    }
    
}

void ConnectRobotAsync(const char* ipAddress, AppLog* log, kuka::EKI* eki)
{
    device::ConnectRobot(ipAddress, log, eki);
}



void configOvPro(int OvPro, kuka::EKI* eki, AppLog* log){
    //std::cout<<"speed:"<<OvPro<<std::endl;
    try{
        kuka::ControlStruct cs;
        cs.control_type_ = kuka::kConfig;
        cs.kv_double_.emplace_back("OvPro", OvPro);
        eki->Write(cs);
        log->AddLog("[info] Change speed: [%02d] \n",OvPro);
    }catch(...){
    log->AddLog("[error] Change speed fail, check robot connect\n");
    }
}
void device::configToolNum(int ToolNum, kuka::EKI* eki, AppLog* log){
    try{
        {
            kuka::ControlStruct cs;
            cs.control_type_ = kuka::kConfig;
            cs.kv_int_.emplace_back("ToolNum", ToolNum);
            eki->Write(cs);
        }
        log->AddLog("[info] Choose ToolNum [%02d] \n", ToolNum);

    }catch(...){
        log->AddLog("[error] Choose ToolNum fail, check robot conncet \n");
    }
}

void device::ShowRobotWindow(bool* connect_device, AppLog* log)
{
    
    int cameraType = -1;
    bool connect_signal = false;
    static char IPAddress[128] = "172.31.1.147";
    std::array<std::string, 2> CameraInfo = {"KUKA", "ABB"};
    

    ImGui::Begin("Connect Robot", connect_device);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
    ImGui::Text("Choose Your Device");
    cameraType = device::ShowRobotSelector("Device##Selector", log);

    ImGui::SeparatorText("Configuration");

    // Show input IP address after choosing camera type
    if (cameraType!=-1){
        ImGui::InputText("IP", IPAddress, IM_ARRAYSIZE(IPAddress));     
    }

    ImGui::Separator();
    ImGui::Spacing();

    // Initialize a eki
    static auto eki = new kuka::EKI;
    // Connection button section
    if (ImGui::Button("Connection"))
    {
        // Use the configre to connect
        std::cout << "Connecting\n";
        log->AddLog("[%s] Connecting %s Robot... IP: '%s'\n",
                 "info", CameraInfo[cameraType].c_str(), IPAddress);
        // device::ConnectRobot(IPAddress, log, eki);
        // connect_signal = true;
        std::thread connectionThread(ConnectRobotAsync, IPAddress, log, eki);
        connectionThread.detach();  

        
    }

    // if (connect_signal){
    //     std::thresad connectionThread(ConnectRobotAsync, IPAddress, log, std::ref(eki));
    //     connectionThread.detach();
    // }

    // Cancle Button section
    if (ImGui::Button("Cancle connection"))
    {
        // if (eki)
        // {
        //     *connect_device = false;
        //     log->AddLog("[%s] Disonnected %s Robot.\n",
        //         "info", CameraInfo[cameraType].c_str());
        //     cameraType = -1;
        //     delete eki;
        // } else {
        //     log->AddLog("Robot did not connected. Please connect first.");
        // }
        *connect_device = false;
        try{
            eki->Disconnect();
        }catch(...){

        }
        log->AddLog("[%s] Disonnected %s Robot.\n",
            "info", CameraInfo[cameraType].c_str());
        cameraType = -1;
        
        isconnectkuka = 0;
    }

    if(cameraType == 0){
        
        //笛卡尔/轴坐标系
        kuka::CoordinateType coordinate_type_ = kuka::kCartesianCoordinateSystem;
        std::array<std::string, 2> coordinateType = {"cartesion", "axis"};
        coordinate_type_ = device::ShowCoordinateSelector("coordinateType##Selector", log);

        //speed % 值1-100
        static int OvPro = 5;
        if(ImGui::SliderInt("Ovpro", &OvPro, 0, 30)){
            configOvPro(OvPro, eki, log);
        }
        
        //工具坐标系
        static int ToolNum = 6;
        if (ImGui::Combo("tool", &ToolNum, "tool_0\0tool_1\0tool_2\0tool_3\0tool_4\0tool_5\0tool_6\0tool_7\0tool_8\0"))
        {
            configToolNum(ToolNum, eki, log);
        }
        //运动类型kLIN,kPTP,kLINRelTool,kPTPRelTool,
        kuka::MovingType moving_type_ = kuka::kPTP;
        std::array<std::string, 4> movingType = {"kLIN", "kLINRelTool","kLINRelTool", "kPTPRelTool"};
        moving_type_ = device::ShowMovingTypeSelector("movingType##Selector", log);

        
        static std::vector<double> position(6, 10.25);

        static char pos1[32] = "0";
        static char pos2[32] = "0";
        static char pos3[32] = "0";
        static char pos4[32] = "0";
        static char pos5[32] = "0";
        static char pos6[32] = "0";

        try{
            position = eki->GetPosition(coordinate_type_);
            if(position[0] != 0){
                snprintf(pos1, sizeof(pos1), "%f", position[0]);
                snprintf(pos2, sizeof(pos2), "%f", position[1]);
                snprintf(pos3, sizeof(pos3), "%f", position[2]);
                snprintf(pos4, sizeof(pos4), "%f", position[3]);
                snprintf(pos5, sizeof(pos5), "%f", position[4]);
                snprintf(pos6, sizeof(pos6), "%f", position[5]);
            }
        }catch(...){

        }
        
        

        static std::vector<double> targetPosition(6);
        //targetPosition = position;
        
        
        targetPosition.clear();
        if(pos1 != nullptr && std::strcmp(pos1, "") != 0 && std::strcmp(pos1, "-") != 0)
            targetPosition.push_back(std::stod(pos1));
        if(pos2 != nullptr && std::strcmp(pos2, "") != 0 && std::strcmp(pos2, "-") != 0)
            targetPosition.push_back(std::stod(pos2));
        if(pos3 != nullptr && std::strcmp(pos3, "") != 0 && std::strcmp(pos3, "-") != 0)
            targetPosition.push_back(std::stod(pos3));
        if(pos4 != nullptr && std::strcmp(pos4, "") != 0 && std::strcmp(pos4, "-") != 0)
            targetPosition.push_back(std::stod(pos4));
        if(pos5 != nullptr && std::strcmp(pos5, "") != 0 && std::strcmp(pos5, "-") != 0)
            targetPosition.push_back(std::stod(pos5));
        if(pos6 != nullptr && std::strcmp(pos6, "") != 0 && std::strcmp(pos6, "-") != 0)
            targetPosition.push_back(std::stod(pos6));
        

        ImGui::InputText("X/A1",     pos1, 32, ImGuiInputTextFlags_CharsDecimal);
        ImGui::SameLine();
        ImGui::PushButtonRepeat(true);
        //if (ImGui::ArrowButton("##left", ImGuiDir_Left)) { counter--; }
        {
            if (ImGui::Button(" X+ "))
            {
                targetPosition[0]= targetPosition[0] + 3;
                //snprintf(pos1, sizeof(pos1), "%f", targetPosition[0]);
                //移动机器人
                seendMoveTarget(eki, log, targetPosition, moving_type_, coordinate_type_);


            }
            ImGui::SameLine();
            if (ImGui::Button(" X- "))
            {
                targetPosition[0]= targetPosition[0] - 3;
                //snprintf(pos1, sizeof(pos1), "%f", targetPosition[0]);
                //移动机器人到 targetPosition
                seendMoveTarget(eki, log, targetPosition, moving_type_, coordinate_type_);

            }
        }

        
        {
            ImGui::InputText("Y/A2",     pos2, 32, ImGuiInputTextFlags_CharsDecimal);
            ImGui::SameLine();
            if (ImGui::Button(" Y+ "))
            {
                targetPosition[1]= targetPosition[1] + 3;
                //snprintf(pos1, sizeof(pos1), "%f", targetPosition[0]);
                //移动机器人
                seendMoveTarget(eki, log, targetPosition, moving_type_, coordinate_type_);
            }
            ImGui::SameLine();
            if (ImGui::Button(" Y- "))
            {
                targetPosition[1]= targetPosition[1] - 3;
                //snprintf(pos1, sizeof(pos1), "%f", targetPosition[0]);
                //移动机器人
                seendMoveTarget(eki, log, targetPosition, moving_type_, coordinate_type_);
            }
        }

        



        {    
            ImGui::InputText("Z/A3",     pos3, 32, ImGuiInputTextFlags_CharsDecimal);
            ImGui::SameLine();
            if (ImGui::Button(" Z+ "))
            {
                targetPosition[2]= targetPosition[2] + 3;
                //snprintf(pos1, sizeof(pos1), "%f", targetPosition[0]);
                //移动机器人
                seendMoveTarget(eki, log, targetPosition, moving_type_, coordinate_type_);
            }
            ImGui::SameLine();
            if (ImGui::Button(" Z- "))
            {
                targetPosition[2]= targetPosition[2] - 3;
                //snprintf(pos1, sizeof(pos1), "%f", targetPosition[0]);
                //移动机器人
                seendMoveTarget(eki, log, targetPosition, moving_type_, coordinate_type_);

            }
        }

        {    
            ImGui::InputText("A/A4",     pos4, 32, ImGuiInputTextFlags_CharsDecimal);
            ImGui::SameLine();
            if (ImGui::Button(" A+ "))
            {
                targetPosition[3]++;
                //snprintf(pos1, sizeof(pos1), "%f", targetPosition[0]);
                //移动机器人
                seendMoveTarget(eki, log, targetPosition, moving_type_, coordinate_type_);

            }
            ImGui::SameLine();
            if (ImGui::Button(" A- "))
            {
                targetPosition[3]--;
                //snprintf(pos1, sizeof(pos1), "%f", targetPosition[0]);
                //移动机器人
                seendMoveTarget(eki, log, targetPosition, moving_type_, coordinate_type_);

            }
        }

        {
            //ImGui::SameLine();
            ImGui::InputText("B/A5",     pos5, 32, ImGuiInputTextFlags_CharsDecimal);
            ImGui::SameLine();
            if (ImGui::Button(" B+ "))
            {
                targetPosition[4]++;
                //snprintf(pos1, sizeof(pos1), "%f", targetPosition[0]);
                //移动机器人
                seendMoveTarget(eki, log, targetPosition, moving_type_, coordinate_type_);

            }
            ImGui::SameLine();
            if (ImGui::Button(" B- "))
            {
                targetPosition[4]--;
                //snprintf(pos1, sizeof(pos1), "%f", targetPosition[0]);
                //移动机器人
                seendMoveTarget(eki, log, targetPosition, moving_type_, coordinate_type_);

            }
        }

        {    //ImGui::SameLine();
            ImGui::InputText("C/A6",     pos6, 32, ImGuiInputTextFlags_CharsDecimal);
            ImGui::SameLine();
            if (ImGui::Button(" C+ "))
            {
                targetPosition[5]++;
                //snprintf(pos1, sizeof(pos1), "%f", targetPosition[0]);
                //移动机器人
                seendMoveTarget(eki, log, targetPosition, moving_type_, coordinate_type_);

            }
            ImGui::SameLine();
            if (ImGui::Button(" C- "))
            {
                targetPosition[5]--;
                //snprintf(pos1, sizeof(pos1), "%f", targetPosition[0]);
                //移动机器人
                seendMoveTarget(eki, log, targetPosition, moving_type_, coordinate_type_);

            }
        }
        ImGui::PopButtonRepeat();
        
        
        
        
        //table
        static int PathNum = 0;

        //path 数据 19*1*6
        static std::vector<std::vector<std::vector<float>>> paths(19, std::vector<std::vector<float>>(1, std::vector<float>(6, 0)));
        //paths.clear();
        //static std::vector<std::vector<std::vector<double>>> paths(19);
        //std::vector<std::vector<double>> mid_path(paths[PathNum], std::vector<double>(6));

        static std::vector<float> f_position(6);
        
        if(ImGui::Button("push position to table")){
            std::cout<<position[0]<<" "
                    <<position[1]<<" "
                    <<position[2]<<" "
                    <<position[3]<<" "
                    <<position[4]<<" "
                    <<position[5]<<std::endl;
            
            f_position.clear();
            f_position.push_back((float)position[0]);
            f_position.push_back((float)position[1]);
            f_position.push_back((float)position[2]);
            f_position.push_back((float)position[3]);
            f_position.push_back((float)position[4]);
            f_position.push_back((float)position[5]);
            if(paths[PathNum][0][0] == 0){
                //paths[PathNum].clear();
            }
            paths[PathNum].push_back(f_position);
            
        }



        //coordinate_type_ 数据

        
        if (ImGui::Combo("path", &PathNum, "path_0\0path_1\0path_2\0path_3\0path_4\0path_5\0path_6\0path_7\0path_8\0path_9\0path_10\0path_11\0path_12\0path_13\0path_14\0path_15\0path_16\0path_17\0path_18\0path_19\0"))
        {

        }
        //std::cout<<"行数：  "<<paths[PathNum].size()<<std::endl;
        //std::cout<<(float)paths[0][0][0]<<"  "<<(float)paths[0][0][5]<<std::endl;
        
        if (ImGui::BeginTable("path", 8, ImGuiTableFlags_Resizable))
        {

            ImGui::TableSetupColumn("X");
            ImGui::TableSetupColumn("Y");
            ImGui::TableSetupColumn("Z");
            ImGui::TableSetupColumn("A");
            ImGui::TableSetupColumn("B");
            ImGui::TableSetupColumn("C");
            ImGui::TableSetupColumn("move_tape");
            ImGui::TableSetupColumn("coordinate_type");
            ImGui::TableHeadersRow();

            // const float TEXT_BASE_WIDTH = ImGui::CalcTextSize("A").x;

            for (int row = 0; row < paths[PathNum].size(); row++)
            {
                ImGui::TableNextRow();
                if (row == 0)
                {
                    // Setup ItemWidth once (instead of setting up every time, which is also possible but less efficient)
                    ImGui::TableSetColumnIndex(0);
                    ImGui::PushItemWidth(-FLT_MIN);
                    ImGui::TableSetColumnIndex(1);
                    //ImGui::PushItemWidth(-ImGui::GetContentRegionAvail().x * 0.5f);
                    ImGui::PushItemWidth(-FLT_MIN);
                    ImGui::TableSetColumnIndex(2);
                    ImGui::PushItemWidth(-FLT_MIN); // Right-aligned
                    ImGui::TableSetColumnIndex(3);
                    ImGui::PushItemWidth(-FLT_MIN); // Right-aligned
                    ImGui::TableSetColumnIndex(4);
                    ImGui::PushItemWidth(-FLT_MIN); // Right-aligned
                    ImGui::TableSetColumnIndex(5);
                    ImGui::PushItemWidth(-FLT_MIN); // Right-aligned
                    ImGui::TableSetColumnIndex(6);
                    ImGui::PushItemWidth(-FLT_MIN); // Right-aligned
                    ImGui::TableSetColumnIndex(7);
                    ImGui::PushItemWidth(-FLT_MIN); // Right-aligned

                    //ImGui::PushItemWidth(-ImGui::GetContentRegionAvail().x * 0.5f);
                }

                
                ImGui::PushID(row);
                ImGui::TableSetColumnIndex(0);
                ImGui::DragFloat("xf0", &paths[PathNum][row][0], 0.005f);

                ImGui::TableSetColumnIndex(1);
                ImGui::DragFloat("xf1", &paths[PathNum][row][1], 0.005f);

                ImGui::TableSetColumnIndex(2);
                ImGui::DragFloat("xf2", &paths[PathNum][row][2], 0.005f);

                ImGui::TableSetColumnIndex(3);
                ImGui::DragFloat("xf3", &paths[PathNum][row][3], 0.005f);

                ImGui::TableSetColumnIndex(4);
                ImGui::DragFloat("xf4", &paths[PathNum][row][4], 0.005f);

                ImGui::TableSetColumnIndex(5);
                ImGui::DragFloat("xf5", &paths[PathNum][row][5], 0.005f);

                /*
                static float vecxyz[4] = { (float)paths[PathNum][row][0], (float)paths[PathNum][row][1], (float)paths[PathNum][row][2], 0.44f };
                //static float vecxyz[4] = {0.1, 0.2, 0.3, 0.4};
                ImGui::InputFloat3("", vecxyz);
                ImGui::SameLine();
                static float vecabc[4] = { (float)paths[PathNum][row][3], (float)paths[PathNum][row][4], (float)paths[PathNum][row][5], 0.44f };
                ImGui::InputFloat3("", vecabc);
                */

                ImGui::TableSetColumnIndex(6);
                moving_type_ = device::ShowMovingTypeSelector("pm1", log);
                ImGui::TableSetColumnIndex(7);
                coordinate_type_ = device::ShowCoordinateSelector("pc1", log);
                

                ImGui::PopID();
            }
            ImGui::EndTable();
        }
        

        if(ImGui::Button("run path")){
            
            if(paths[PathNum][0][0] == 0){
                log->AddLog("[%s] target position error.\n",
                    "error");
            }else{
                path_m.clear();
                for(int i = 0; i < paths[PathNum].size(); i++){
                    
                    if(paths[PathNum][i][0] == 0 
                    || paths[PathNum][i][1] == 0 
                    || paths[PathNum][i][2] == 0 
                    || paths[PathNum][i][3] == 0 
                    || paths[PathNum][i][4] == 0 
                    || paths[PathNum][i][5] == 0){
                        continue;
                    }

                    static std::vector<double> tar(6);
                    tar.clear();
                    tar.push_back((double)paths[PathNum][i][0]);
                    tar.push_back((double)paths[PathNum][i][1]);
                    tar.push_back((double)paths[PathNum][i][2]);
                    tar.push_back((double)paths[PathNum][i][3]);
                    tar.push_back((double)paths[PathNum][i][4]);
                    tar.push_back((double)paths[PathNum][i][5]);

                    path_m.push_back(tar);

                }
                isrunpath++;
                std::cout<<isrunpath<<std::endl;
            }
            
        }
        
        if(ImGui::Button("save path to txt")){

            std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
            time_t timestamp = std::chrono::system_clock::to_time_t(now);

            std::ofstream file_out_mid("output/path/path" + std::to_string(timestamp) + ".txt"); // 创建一个名为"data.txt"的文档
            if (file_out_mid.is_open()) {
                log->AddLog("[%s] open path file.\n",
                    "info");
            } else {
                log->AddLog("[%s] Failed to open the path file..\n",
                    "error");
            }
            
            if(paths[PathNum][0][0] == 0){
                log->AddLog("[%s] target position error.\n",
                    "error");
            }else{
                std::cout<<"轨迹点数： "<<paths[PathNum].size()<<std::endl;
                for(int i = 0; i < paths[PathNum].size(); i++){
                    
                    if(paths[PathNum][i][0] == 0 
                    || paths[PathNum][i][1] == 0 
                    || paths[PathNum][i][2] == 0 
                    || paths[PathNum][i][3] == 0 
                    || paths[PathNum][i][4] == 0 
                    || paths[PathNum][i][5] == 0){
                        log->AddLog("[%s] target position [%d] error.\n",
                                    "error", i);
                        continue;
                    }
                    file_out_mid << paths[PathNum][i][0]<<" "
                                << paths[PathNum][i][1]<<" "
                                << paths[PathNum][i][2]<<" "
                                << paths[PathNum][i][3]<<" "
                                << paths[PathNum][i][4] <<" "
                                << paths[PathNum][i][5] << std::endl; 
                }
            }
            
        }
        static std::string filename;


        static char str0[128] = "/home/shibo/Document/saiwider_vision_imgui/src/builds/output/path/path1712909959.txt";
        ImGui::InputText("path name", str0, IM_ARRAYSIZE(str0));
        filename = str0;
        if(ImGui::Button("choose path txt file to run")){

            
            /*
            static ImGui::FileBrowser pathfilebrowser;
            pathfilebrowser.Display;
            static std::string filename;
            if(file_browser.HasSelected()){
                filename = file_browser.GetSelected().string();
                std::cout<<filename<<std::endl;
            }
            */


            {

                //std::ifstream file(image_dir / "roboinfo.txt" , std::ios::app); // 追加模式打开文件
                std::ifstream file(filename, std::ios::app); // 追加模式打开文件
                if (file.is_open()) { 
                    std::cout << "已创建文件"+ filename +"\n";
                } else {
                    std::cout << "无法打开文件\n";
                    return ;
                }

                
                path_m.clear();
                std::vector<double> point;

                
                std::string s;//我创建的变量，存储数据用的
                double doubleArray[6];
                int i = 0;
                while(file>>s){//输入文件流
                    doubleArray[i] = std::stod(s);
                    i++;
                    if (i == 6)
                    {
                        point.clear();
                        i = 0;
                        point.push_back(doubleArray[0]);
                        point.push_back(doubleArray[1]);
                        point.push_back(doubleArray[2]);
                        point.push_back(doubleArray[3]);
                        point.push_back(doubleArray[4]);
                        point.push_back(doubleArray[5]);
                        path_m.push_back(point);
                        std::cout<<point[0]<<" "
                                <<point[1]<<" "
                                <<point[2]<<" "
                                <<point[3]<<" "
                                <<point[4]<<" "
                                <<point[5]<<" "<<std::endl;
                    }
                }
                

            }
            isrunpath++;
            
        }

        

    }
        
    ImGui::End();
}
