# 设置 CMake 版本要求
cmake_minimum_required(VERSION 3.24)

# 项目名称
project(kuka_eki)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置编译选项，开启所有警告并将警告视为错误
add_compile_options("-Wall")
set(CMAKE_COMPILE_WARNING_AS_ERROR ON)

# 查找 Boost 库，指定需要的组件
find_package(Boost REQUIRED COMPONENTS system)

# 使用 PkgConfig 查找依赖包 "tinyxml"
find_package(PkgConfig REQUIRED)
set(DEPENDENCIES "tinyxml")
pkg_check_modules(DEPS REQUIRED IMPORTED_TARGET ${DEPENDENCIES})

# 包含目录
include_directories(
    include
)

# 创建静态库 kuka_eki_hw_interface
add_library(
    kuka_eki STATIC
    kuka_eki_hw_interface.cpp
)

# 链接静态库和依赖库
target_link_libraries(
    kuka_eki
    PUBLIC PkgConfig::DEPS
    -pthread
)

# 安装目标库到指定的目录
install(TARGETS kuka_eki DESTINATION lib)

# 安装头文件目录到指定的目录
install(DIRECTORY include/kuka_eki_hw_interface DESTINATION include)


