cmake_minimum_required(VERSION 3.8)
project(system_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(state_interfaces REQUIRED)
find_package(jsoncpp REQUIRED)


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${JSON_INC_PATH})

# [Node] system_manager : 编译 链接 添加ros依赖 安装
set(NODE_NAME system_manager)
add_executable(${NODE_NAME} 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/system_manager.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/system_manager_node.cpp
)
target_link_libraries(${NODE_NAME} jsoncpp_lib)
ament_target_dependencies(${NODE_NAME} rclcpp std_msgs state_interfaces)
install(TARGETS ${NODE_NAME}  DESTINATION lib/${PROJECT_NAME})


# [Node] test_system_manager : 编译 链接 添加ros依赖 安装
set(NODE_NAME test_system_manager)
add_executable(${NODE_NAME} 
  ${CMAKE_CURRENT_SOURCE_DIR}/test/test_system_manager.cpp
)
ament_target_dependencies(${NODE_NAME} rclcpp std_msgs state_interfaces)
install(TARGETS ${NODE_NAME}  DESTINATION lib/${PROJECT_NAME})

ament_package()
