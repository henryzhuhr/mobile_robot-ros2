cmake_minimum_required(VERSION 3.5)
project(cpp_video_streamer)

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

# ========================
# find dependencies
# ========================
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(interfaces REQUIRED) # 自定义消息接口

find_package(OpenCV COMPONENTS)
if( OpenCV_FOUND )
  # message(" -- OpenCV found:")
  # message("    -- version  :" ${OpenCV_VERSION})
  # message("    -- include  :" ${OpenCV_INCLUDE_DIRS})
  # message("    -- libraries:" ${OpenCV_LIBRARIES})
  include_directories(${OpenCV_INCLUDE_DIRS})
  link_directories(${OpenCV_LIBRARY_DIRS})
  add_definitions(${OpenCV_DEFINITIONS})
else()
	message(FATAL_ERROR "-- didn't find OpenCV on system, disabling OpenCV")
endif()


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# [Node] video_reader : 编译 链接 添加ros依赖 安装
add_executable(video_reader 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/video_reader.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/node_video_reader.cpp
)
target_link_libraries(video_reader ${OpenCV_LIBRARIES})
ament_target_dependencies(video_reader rclcpp std_msgs sensor_msgs)
install(TARGETS video_reader DESTINATION lib/${PROJECT_NAME})

# [Node] video_viewer : 编译 链接 添加ros依赖 安装
add_executable(video_viewer 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/video_viewer.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/node_video_viewer.cpp
)
target_link_libraries(video_viewer ${OpenCV_LIBRARIES})
ament_target_dependencies(video_viewer 
  rclcpp 
  std_msgs 
  sensor_msgs 
  interfaces # 自定义消息接口
)
install(TARGETS video_viewer DESTINATION lib/${PROJECT_NAME})

# install launch
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
