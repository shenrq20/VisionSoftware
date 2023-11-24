cmake_minimum_required(VERSION 3.22)

project(VisionSoftware LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_GENERATOR Ninja)

set(CMAKE_PREFIX_PATH "/usr/local/lib/cmake;${CMAKE_PREFIX_PATH}")

# find_package()

# include_directories()

# 添加可执行文件 
add_executable(VisionSoftware main.cpp)