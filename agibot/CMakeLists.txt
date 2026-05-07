cmake_minimum_required(VERSION 3.8)

project(agibot)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 识别 x86_64 架构（兼容不同系统的返回值）
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64|amd64")
    set(ARCH "x86_64")
# 识别 arm64/aarch64 架构
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(ARCH "aarch64")
else()
    message(FATAL_ERROR "不支持的架构: ${CMAKE_HOST_SYSTEM_PROCESSOR}，请检查 lib 目录下是否有对应架构的库")
endif()

message(STATUS "检测到系统架构: ${ARCH}")

# 库文件路径：上层目录 -> lib/zsl-1/架构目录
set(LIB_PATH "${CMAKE_SOURCE_DIR}/lib/${ARCH}")
message(STATUS "将链接的库路径: ${LIB_PATH}")

# 查找对应架构的库（库文件名为 libmc_sdk_zsl_1_${ARCH}.so）
find_library(MC_SDK_LIB 
    NAMES "mc_sdk_zsl_1_${ARCH}"  # 库名（不含前缀lib和后缀.so）
    PATHS ${LIB_PATH}
    NO_DEFAULT_PATH  # 只在指定路径查找，避免找到其他版本
    REQUIRED
)

# 验证库是否找到
if(NOT MC_SDK_LIB)
    message(FATAL_ERROR "在 ${LIB_PATH} 中未找到库文件 mc_sdk_zsl_1_${ARCH}.so，请检查库是否存在")
else()
    message(STATUS "找到库文件: ${MC_SDK_LIB}")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)


add_executable(agibot_move_sub src/agibot.cpp)
# 新增：设置运行时库搜索路径（RPATH）
set_target_properties(agibot_move_sub PROPERTIES
    # 运行时优先查找编译时的库路径
    INSTALL_RPATH_USE_LINK_PATH TRUE
    # 手动指定运行时库路径（替换为实际LIB_PATH）
    INSTALL_RPATH "${LIB_PATH}"
)
target_link_libraries(agibot_move_sub ${MC_SDK_LIB})
target_include_directories(agibot_move_sub PRIVATE 
    ${CMAKE_SOURCE_DIR}/lib

    ${CMAKE_SOURCE_DIR}/include  # 公共头文件
)
ament_target_dependencies(agibot_move_sub rclcpp geometry_msgs std_msgs)


install(TARGETS 

  agibot_move_sub
  
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
        FILES_MATCHING PATTERN "*.launch.py")


ament_package()

