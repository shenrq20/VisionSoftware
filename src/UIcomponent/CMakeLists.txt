cmake_minimum_required(VERSION 3.26)
project(UIcomponent)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dear ImGui
set(IMGUI_DIR ../../3rdparty/imgui)
include_directories(${IMGUI_DIR} ${IMGUI_DIR}/backends include)

add_library(UIcomponent STATIC log.cpp ${IMGUI_DIR}/imgui.cpp)
