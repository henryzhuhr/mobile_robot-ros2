cmake_minimum_required(VERSION 3.5)
project(cpp_ju_video)

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

add_definitions(-DROS2 -DROS_DISTRO=${ROS_DISTRO})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(jetson-utils REQUIRED)
include_directories(${jetson-utils})
find_package(CUDA REQUIRED)
message(" -- CUDA version: ${CUDA_VERSION}")
message(" -- CUDA_INCLUDE_DIRS: ${CUDA_INCLUDE_DIRS}")
message(" -- CUDA_LIBRARIES:    ${CUDA_LIBRARIES}")
include_directories(${CUDA_INCLUDE_DIRS})
# detect ROS version
string(TOUPPER $ENV{ROS_DISTRO} ROS_DISTRO)
set(ROS_DISTRO "ROS_${ROS_DISTRO}")
message("detected ROS_DISTRO=${ROS_DISTRO}")


# header
include_directories(include)


# sources shared across nodes
set(common_src src/image_converter.cpp src/ros_compat.cpp)


# 可执行文件并将其命名为 listener ，以便您可以使用 ros2 run 运行节点
add_executable(video_reader src/node_video_reader.cpp ${common_src})
target_link_libraries(video_reader ${CUDA_LIBRARIES} ${JETSON_UTILS_HOME}/lib/libjetson-utils.so)
ament_target_dependencies(video_reader rclcpp std_msgs sensor_msgs)
install(TARGETS video_reader DESTINATION lib/${PROJECT_NAME})

# 可执行文件并将其命名为 listener ，以便您可以使用 ros2 run 运行节点
# add_executable(video_viewer src/node_video_viewer.cpp ${common_src})
# target_link_libraries(video_viewer)
# ament_target_dependencies(video_viewer rclcpp std_msgs sensor_msgs)
# install(TARGETS video_viewer DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_package()

