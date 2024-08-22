cmake_minimum_required(VERSION 3.8)
project(motion_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(state_interfaces REQUIRED)
find_package(system_state REQUIRED)
find_package(jsoncpp REQUIRED)
# find_package(serial REQUIRED)   # 串口

# if (system_state_FOUND_AMENT_PACKAGE)
#   message(FATAL_ERROR "${system_state_INCLUDE_DIRS}")
# endif()


include_directories(${system_state_INCLUDE_DIRS}/include) # install/system_state/include/include 被嵌套了一层
include_directories(include)
include_directories(${JSON_INC_PATH})


# [Node] motion_controller : 编译 链接 添加ros依赖 安装
set(executable_name motion_controller_node)
add_executable(${executable_name} 
  src/motion_serial.cpp src/motion_controller__serial.cpp
  src/motion_controller.cpp
  app/motion_controller_node.cpp
)
ament_target_dependencies(${executable_name}
  rclcpp std_msgs
  state_interfaces system_state
)
target_link_libraries(${executable_name} 
  serial
  jsoncpp_lib
)
install(
  TARGETS ${executable_name}  
  DESTINATION lib/${PROJECT_NAME}
)



ament_package()
