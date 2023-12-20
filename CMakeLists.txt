cmake_minimum_required(VERSION 3.24)

project(VisionSoftware)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_GENERATOR Ninja)

# 寻找并包含 Eigen 库的头文件目录
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# 设置 OpenVDB 模块路径
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/lib/cmake/OpenVDB")

# 寻找并包含 OpenCV 库的头文件目录
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# 寻找 VTK 库
find_package(VTK REQUIRED)
#include(${VTK_USE_FILE})

# 添加可执行文件 
add_executable(VisionSoftware main.cpp)

# 链接所需的库
target_link_libraries(VisionSoftware ${VTK_LIBRARIES})
target_link_libraries(VisionSoftware kuka_eki_hw_interface)
target_include_directories(
    VisionSoftware 
    PUBLIC src/device/kuka_eki_hw_interface/include
    PUBLIC src/device/kuka_eki_hw_interface/include/kuka_eki_hw_interface
    )

# 添加子目录
add_subdirectory(src)