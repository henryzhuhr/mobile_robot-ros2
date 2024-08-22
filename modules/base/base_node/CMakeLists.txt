cmake_minimum_required(VERSION 3.20)
project(base_node)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(system_state REQUIRED)
find_package(state_interfaces REQUIRED)


include_directories(include)
include_directories(${system_state_INCLUDE_DIRS}/include) # install/system_state/include/include 被嵌套了一层

add_library(${PROJECT_NAME} SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/src/base_node.cpp
)
ament_target_dependencies(${PROJECT_NAME} rclcpp state_interfaces system_state)
install(
  TARGETS ${PROJECT_NAME} 
  LIBRARY DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)
ament_export_include_directories(include)
ament_export_include_directories(${system_state_INCLUDE_DIRS}/include)
ament_export_libraries(${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# [Node] system_manager : 编译 链接 添加ros依赖 安装
# set(NODE_NAME base_node_node)
# add_executable(${NODE_NAME} 
#   ${CMAKE_CURRENT_SOURCE_DIR}/src/base_node.cpp
#   ${CMAKE_CURRENT_SOURCE_DIR}/app/base_node_main.cpp
# )
# # target_link_libraries(${NODE_NAME} jsoncpp_lib)
# ament_target_dependencies(${NODE_NAME} rclcpp state_interfaces system_state)
# install(TARGETS ${NODE_NAME} DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()




ament_package()
