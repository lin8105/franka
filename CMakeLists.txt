cmake_minimum_required(VERSION 3.10)
project(Multimodal_Trajectory_Recorder)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Franka REQUIRED)
find_package(Threads REQUIRED)
# 【新增】：查找 ZLIB 库（cnpy 依赖它进行 numpy 数据的底层压缩）
find_package(ZLIB REQUIRED)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/common
)

# 【修改】：将 cnpy.cpp 添加到可执行文件的编译列表中
add_executable(path_recorder
    path_recorder_controller.cpp
    common/examples_common.cpp
    FrankaRobot.cpp
    cnpy.cpp
)

# 【修改】：链接 ZLIB 库
target_link_libraries(path_recorder
    PRIVATE
    Franka::Franka
    Eigen3::Eigen
    Threads::Threads
    dqrobotics  
    ZLIB::ZLIB
)
