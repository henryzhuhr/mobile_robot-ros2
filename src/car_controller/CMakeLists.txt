cmake_minimum_required(VERSION 3.5)
project(car_controller)

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
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(include) # 包含头文件

# --------------------------------------------
# [Node] talker : 编译 链接ROS 安装
set(node_name control_car)
add_executable(${node_name} # 可执行文件名，也就是节点(运行的文件)名
  src/control_car.cpp # 源文件
  src/CarController.cpp
  src/SerialController.cpp
)
# target_link_libraries(video_reader ${OpenCV_LIBRARIES}) # 如果有
ament_target_dependencies(${node_name}
  rclcpp    # 添加 ROS 包: rclcpp
  std_msgs  # 添加 ROS 包: std_msgs
)
install(TARGETS     ${node_name}              # 安装的可执行文件
        DESTINATION lib/${PROJECT_NAME} # 安装的位置
)



ament_package()
