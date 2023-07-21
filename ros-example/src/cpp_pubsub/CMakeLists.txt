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

# [Node] talker : 编译 链接ROS 安装
add_executable(talker # 可执行文件名，也就是节点(运行的文件)名
  src/publisher_member_function.cpp # 源文件
)
# target_link_libraries(video_reader ${OpenCV_LIBRARIES}) # 如果有
ament_target_dependencies(talker
  rclcpp    # 添加 ROS 包: rclcpp
  std_msgs  # 添加 ROS 包: std_msgs
)
install(TARGETS     talker              # 安装的可执行文件
        DESTINATION lib/${PROJECT_NAME} # 安装的位置
)

# [Node] listener : 编译 链接ROS 安装
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)
install(TARGETS listener DESTINATION lib/${PROJECT_NAME})


ament_package()
