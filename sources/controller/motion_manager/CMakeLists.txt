cmake_minimum_required(VERSION 3.8)
project(motion_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(state_interfaces REQUIRED)
find_package(jsoncpp REQUIRED)
# find_package(serial REQUIRED)   # 串口


if (EXISTS serial)
  add_subdirectory(serial)
else()
  message(FATAL_ERROR  "run \"git clone https://github.com/HenryZhuHR/serial.git sources/controller/motion_manager/serial\" first")
endif()


# export WORKSPACE=$PWD
include_directories($ENV{WORKSPACE}/sources/common/system_state/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${JSON_INC_PATH})


# [Node] motion_manager : 编译 链接 添加ros依赖 安装
set(NODE_NAME motion_manager)
add_executable(${NODE_NAME} 
  ${CMAKE_CURRENT_SOURCE_DIR}/app/motion_manager.cpp
)
target_link_libraries(${NODE_NAME} 
  serial
  jsoncpp_lib
)

ament_target_dependencies(${NODE_NAME} rclcpp std_msgs state_interfaces)
install(TARGETS ${NODE_NAME}  DESTINATION lib/${PROJECT_NAME})



ament_package()
