cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)# 添加依赖
find_package(std_msgs REQUIRED)# 添加依赖

# 可执行文件并将其命名为 talker ，以便您可以使用 ros2 run 运行节点
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

# 可执行文件并将其命名为 listener ，以便您可以使用 ros2 run 运行节点
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)



# 添加 “安装 (目标……) `` section so `` ros2运行” 可以找到您的可执行文件
install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()
