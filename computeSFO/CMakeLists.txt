cmake_minimum_required(VERSION 3.9)
project(computeClockOffset LANGUAGES C)

set(CMAKE_C_STANDARD 11)

# 设置输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

message(STATUS "Current source directory: ${CMAKE_CURRENT_SOURCE_DIR}")

# 源文件列表
aux_source_directory(src SRC_LIST)

# 包含目录
include_directories(include)

# MATLAB 路径（请根据您的安装路径调整）
# set(MATLAB_ROOT "/home/amumu/MATLAB/R2024a")
# set(MATLAB_INCLUDE_DIR "${MATLAB_ROOT}/extern/include")
# set(MATLAB_LIB_DIR "~/MATLAB/R2024a/bin/glnxa64")
set(MATLAB_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib")

# 添加包含目录
# include_directories(${MATLAB_INCLUDE_DIR})

# 添加可执行目标
add_executable(${PROJECT_NAME} ${SRC_LIST})

# 链接所需的 MATLAB 库
target_link_libraries(${PROJECT_NAME} m)