#include <array>
#include <iostream>
#include "imgui.h"
#include "devCamConnect.h"
#include "log.h"
#include "genicam.h"
#include <thread>
#include "gvalue.h"
#include <chrono>





// bool ImGui::ShowStyleSelector(const char* label)
int device::ShowCameraSelector(const char* label, AppLog* log)
{
    static int device_idx = -1;
    if (ImGui::Combo(label, &device_idx, "HiKang\0Sick\0"))
    {
        switch (device_idx)
        {
        case 0: log->AddLog("Configure HiKang Camera\n"); break; // Call configure function
        case 1: log->AddLog("Configure Sick Camera\n"); break;   // Call configure function
        }
    }
    return device_idx;
}



void device::ConnectCam(const char* ip_address, AppLog* log)
{

    // # 创建并连接相机 
    auto camera = new GENICAM(ip_address);
    camera->Connect();
    log->AddLog("[info] Camera connected.\n");
    isconnect = 1;

    
    std::cout<<"开始值："<<get_picture<<std::endl;
    while(isconnect){

        if(get_picture > 0){
            std::cout<<"值："<<get_picture<<std::endl;
            device::getHikangPicture(camera, log);
            get_picture--;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    camera->Disconnect();
    isconnect = 0;
    

}


void ConnectCamAsync(const char* ipAddress, AppLog* log)
{
    device::ConnectCam(ipAddress, log);
}



void device::ShowCameraWindow(bool* connect_device, AppLog* log)
{
    int cameraType = -1;
    static char IPAddress[128] = "172.31.1.112";
    std::array<std::string, 2> CameraInfo = {"HiKang", "Sick"};
    


    ImGui::Begin("Connect Camera", connect_device);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
    ImGui::Text("Choose Your Device");
    cameraType = device::ShowCameraSelector("Device##Selector", log);

    ImGui::SeparatorText("Configuration");

    if (cameraType==0){
        // HiKang config
        ImGui::InputText("IP", IPAddress, IM_ARRAYSIZE(IPAddress));     
    } else if (cameraType==1) {
        //Sick config
        ImGui::InputText("IP", IPAddress, IM_ARRAYSIZE(IPAddress));
    }

    ImGui::Separator();
    ImGui::Spacing();

    if (ImGui::Button("Connection"))
    {
        // Use the configre to connect
        std::cout << "Connecting\n";
        log->AddLog("[%s] Connecting %s Camera... IP: '%s'\n",
                 "info", CameraInfo[cameraType].c_str(), IPAddress);

        if(CameraInfo[cameraType].c_str()[0] == 'H'){
            //creat thread
            std::thread connectionCamThread(ConnectCamAsync, IPAddress, log);
            connectionCamThread.detach();
        }
        
    }

    

    if(cameraType == 0)
    {
        if (ImGui::Button("Cancle connection"))
        {
            *connect_device = false;
            log->AddLog("[%s] Disonnected %s Camera.\n",
                "info", CameraInfo[cameraType].c_str());
            cameraType = -1;
            isconnect = 0;
        }
        
        if (ImGui::Button("take picture"))
        {
            std::cout<<"will take pictures!"<<std::endl;
            get_picture++;
        }

        static cv::Mat picture = cv::imread("../../img/laser_test.png");

        while (hikangPictureQueue.size() > 0)
        {
            picture = hikangPictureQueue.front();
            hikangPictureQueue.pop();
        }
        /*
        // 将 cv::Mat 转换为 ImGui 可以理解的纹理数据格式
        GLuint textureId;
        glGenTextures(1, &textureId);
        glBindTexture(GL_TEXTURE_2D, textureId);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, picture.cols, picture.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, picture.data);
        glBindTexture(GL_TEXTURE_2D, 0);

        // 在 ImGui 窗口中显示图像
        ImVec2 imageSize(picture.cols, picture.rows);
        ImGui::Image(reinterpret_cast<ImTextureID>(static_cast<intptr_t>(textureId)), imageSize);
        */
        /*
        // 将图像数据绑定到 ImGui 的纹理中
        ImTextureID textureID = ImGui::GetIO().Fonts->TexID;  // 获取 ImGui 的默认纹理 ID
        glBindTexture(GL_TEXTURE_2D, (GLuint)(intptr_t)textureID);  // 将纹理绑定到当前活动的 OpenGL 纹理单元
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, picture.cols, picture.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, picture.data);  // 将图像数据上传到纹理

        // 在 ImGui 窗口中显示图像
        ImGui::Image(textureID, ImVec2(picture.cols, picture.rows));
        */
        /*
        ImGuiIO& io = ImGui::GetIO();
        ImTextureID my_tex_id = io.Fonts->TexID;
        float my_tex_w = (float)io.Fonts->TexWidth;
        float my_tex_h = (float)io.Fonts->TexHeight;
        {
            static bool use_text_color_for_tint = false;
            ImGui::Checkbox("Use Text Color for Tint", &use_text_color_for_tint);
            ImGui::Text("%.0fx%.0f", my_tex_w, my_tex_h);
            ImVec2 pos = ImGui::GetCursorScreenPos();
            ImVec2 uv_min = ImVec2(0.0f, 0.0f);                 // Top-left
            ImVec2 uv_max = ImVec2(1.0f, 1.0f);                 // Lower-right
            ImVec4 tint_col = use_text_color_for_tint ? ImGui::GetStyleColorVec4(ImGuiCol_Text) : ImVec4(1.0f, 1.0f, 1.0f, 1.0f); // No tint
            ImVec4 border_col = ImGui::GetStyleColorVec4(ImGuiCol_Border);
            ImGui::Image(my_tex_id, ImVec2(my_tex_w, my_tex_h), uv_min, uv_max, tint_col, border_col);
            
        }*/



        if (ImGui::Button("save picture"))
        {
            std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
            time_t timestamp = std::chrono::system_clock::to_time_t(now);
            std::string output_name =std::to_string(timestamp);
            std::cout<<"will take pictures!"<<std::endl;
            cv::imwrite("output/photo/ori_image_" + output_name + ".png", picture);

        }
    }

    
    ImGui::End();
}


void device::getHikangPicture(GENICAM * camera, AppLog* log)
{   
    try {
        hikangPictureQueue.push(camera->GetImage());
        log->AddLog("[%s] Hikang camera took one picture.\n",
            "info");
    } catch (const std::exception& e) {
        log->AddLog("[%s] Hikang camera took fail.\n",
            "warn");
        isconnect = 0;
        log->AddLog("[%s] Please reconnect Hikang camera.\n",
            "warn");
    }
    
}
