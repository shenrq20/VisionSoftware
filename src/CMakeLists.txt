# Example usage:
#  mkdir build
#  cd build
#  cmake -g "Visual Studio 14 2015" ..

cmake_minimum_required(VERSION 3.26)
project(SaiwiderVision C CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DVK_PROTOTYPES")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DVK_PROTOTYPES")

# GLFW
set(GLFW_DIR ../3rdparty/glfw) # Set this to point to an up-to-date GLFW repo
option(GLFW_BUILD_EXAMPLES "Build the GLFW example programs" OFF)
option(GLFW_BUILD_TESTS "Build the GLFW test programs" OFF)
option(GLFW_BUILD_DOCS "Build the GLFW documentation" OFF)
option(GLFW_INSTALL "Generate installation target" OFF)
option(GLFW_DOCUMENT_INTERNALS "Include internals in documentation" OFF)
add_subdirectory(${GLFW_DIR} binary_dir EXCLUDE_FROM_ALL)
include_directories(${GLFW_DIR}/include)

# Dear ImGui
set(IMGUI_DIR ../3rdparty/imgui)
include_directories(${IMGUI_DIR} ${IMGUI_DIR}/backends ..)

# kuka_eki_hw_interface
set(KUKA_DIR device/kuka_eki_hw_interface)
include_directories(${IMGUI_DIR} ${IMGUI_DIR}/include ..)

# 寻找Eigen库并包含头文件目录
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


# 寻找OpenVDB并设置模块路径
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/lib/cmake/OpenVDB")

# 寻找PCL库并包含头文件目录
find_package(PCL 1.10 REQUIRED COMPONENTS)
include_directories(${PCL_INCLUDE_DIRS})


# 设置PCL库文件的路径
link_directories(${PCL_LIBRARY_DIRS})

# 添加PCL定义
add_definitions(${PCL_DEFINITIONS})

# 寻找OpenCV库并包含头文件目录
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


# 查找OpenGL库
find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS})

# 寻找VTK库
find_package(VTK REQUIRED)
#include(${VTK_USE_FILE})


# aravis
include_directories(/usr/local/include/aravis-0.8)
# glib
include_directories(/usr/include/glib-2.0)
include_directories(/usr/lib/x86_64-linux-gnu/glib-2.0/include)
# globalvalue
include_directories(glovalue/include)


# Libraries
find_package(Vulkan REQUIRED)
#find_library(VULKAN_LIBRARY
  #NAMES vulkan vulkan-1  )
#set(LIBRARIES "glfw;${VULKAN_LIBRARY}")
set(LIBRARIES "glfw;Vulkan::Vulkan")

# Use vulkan headers from glfw:
include_directories(${GLFW_DIR}/deps)

# Use deviceConnect
set(DEVICE_DIR deviceConnect/include)
include_directories(${DEVICE_DIR})

# Use UIcomponent
set(UICOMPONENT_DIR UIcomponent/include)
include_directories(${UICOMPONENT_DIR})

# Use device
set(DEVICE_DIR device)
include_directories(${DEVICE_DIR}/kuka_eki_hw_interface/include/kuka_eki_hw_interface)
include_directories(${DEVICE_DIR}/genicam/include)
include_directories(${DEVICE_include_directoriesDIR}/image_processing/include)
# set(device device/kuka_eki_hw_interface)
# include_directories(${KUKA_EKI_DIR}/include/kuka_eki_hw_interface)

file(GLOB sources *.cpp)

add_executable(SaiwiderVision main.cpp ${sources} ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp ${IMGUI_DIR}/backends/imgui_impl_vulkan.cpp ${IMGUI_DIR}/imgui.cpp ${IMGUI_DIR}/imgui_draw.cpp ${IMGUI_DIR}/imgui_demo.cpp ${IMGUI_DIR}/imgui_tables.cpp ${IMGUI_DIR}/imgui_widgets.cpp)
target_link_libraries(SaiwiderVision ${LIBRARIES} ${VTK_LIBRARIES} ${PCL_LIBRARIES} ${OPENGL_LIBRARIES} UIcomponent deviceConnect kuka_eki genicam glovalue)
target_compile_definitions(SaiwiderVision PUBLIC -DImTextureID=ImU64)




add_subdirectory(deviceConnect)
add_subdirectory(UIcomponent)
add_subdirectory(device)
add_subdirectory(glovalue)



