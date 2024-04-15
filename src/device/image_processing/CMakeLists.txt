cmake_minimum_required(VERSION 3.24)
project(image_processing)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options("-Wall")
set(CMAKE_COMPILE_WARNING_AS_ERROR ON)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(imgproc STATIC imgproc.cpp)
target_link_libraries(imgproc PUBLIC ${OpenCV_LIBS})
