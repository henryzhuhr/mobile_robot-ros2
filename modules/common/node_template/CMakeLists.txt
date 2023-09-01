cmake_minimum_required(VERSION 3.8)
project(node_template)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(system_state REQUIRED)
find_package(state_interfaces REQUIRED)
find_package(base_node REQUIRED)


# [Node] 编译 链接 添加ros依赖 安装
set(NODE_NAME ${PROJECT_NAME})
add_executable(${NODE_NAME}  
  ${CMAKE_CURRENT_SOURCE_DIR}/app/node_template.cpp
)
# target_link_libraries(${NODE_NAME} jsoncpp_lib)
ament_target_dependencies(${NODE_NAME}
  rclcpp
  system_state state_interfaces
  base_node
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
