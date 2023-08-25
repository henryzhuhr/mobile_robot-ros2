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


if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/serial)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/serial)
else()
  message(FATAL_ERROR  "run \"git clone https://github.com/HenryZhuHR/serial.git \\ sources/controller/motion_manager/serial\" first")
endif()


# export WORKSPACE=$PWD
include_directories($ENV{WORKSPACE}/modules/common/system_state/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${JSON_INC_PATH})


# [Node] motion_manager : 编译 链接 添加ros依赖 安装
set(NODE_NAME motion_manager_node)
add_executable(${NODE_NAME} 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/motion_serial.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/motion_manager__serial.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/motion_manager.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/app/${NODE_NAME}.cpp
)
target_link_libraries(${NODE_NAME} 
  serial
  jsoncpp_lib
)

ament_target_dependencies(${NODE_NAME} rclcpp std_msgs state_interfaces)
install(TARGETS ${NODE_NAME}  DESTINATION lib/${PROJECT_NAME})



ament_package()
