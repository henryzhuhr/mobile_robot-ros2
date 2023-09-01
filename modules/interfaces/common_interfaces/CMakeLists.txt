cmake_minimum_required(VERSION 3.5)
project(common_interfaces)

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
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)# 消息依赖
find_package(sensor_msgs REQUIRED)# 消息依赖


rosidl_generate_interfaces(${PROJECT_NAME}
  # "msg/Lanes.msg" # 车道线消息格式
  # "msg/ImageViewer.msg" # 
  # # 车辆状态
  # "msg/Location.msg" # 定位信息
  # "msg/Speed.msg" # 速度状态
  # "msg/RuntimeState.msg" # 运行状态
  "msg/Gyroscope.msg" # 陀螺仪数据
  "msg/Acceleration.msg" # 加速度计数据
  "msg/Attitude.msg" # 加速度计数据
  "msg/Imu6Axis.msg" # 6轴IMU数据
  DEPENDENCIES std_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
