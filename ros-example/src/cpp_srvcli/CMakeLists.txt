cmake_minimum_required(VERSION 3.5)
project(cpp_srvcli)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_interfaces REQUIRED)

# [Node] server : 编译、链接、安装
add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server rclcpp custom_interfaces)
install(TARGETS server DESTINATION lib/${PROJECT_NAME})

# [Node] client : 编译、链接、安装
add_executable(client src/add_two_ints_client.cpp)
ament_target_dependencies(client rclcpp custom_interfaces)
install(TARGETS client DESTINATION lib/${PROJECT_NAME})

ament_package()
