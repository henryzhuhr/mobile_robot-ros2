cmake_minimum_required(VERSION 3.8)
project(base_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)


include_directories(include)

# [Node] system_manager : 编译 链接 添加ros依赖 安装
set(NODE_NAME ${PROJECT_NAME})
add_executable(${NODE_NAME} 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/config_parse.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/system_manager.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/app/system_manager_node.cpp
)
target_link_libraries(${NODE_NAME} jsoncpp_lib)
ament_target_dependencies(${NODE_NAME} rclcpp std_msgs state_interfaces)
install(TARGETS ${NODE_NAME}  DESTINATION lib/${PROJECT_NAME})

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
