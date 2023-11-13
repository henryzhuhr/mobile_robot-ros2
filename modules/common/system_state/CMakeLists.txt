cmake_minimum_required(VERSION 3.20)
project(system_state)

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
find_package(std_msgs REQUIRED)

find_package(state_interfaces REQUIRED)


# include_directories(include)

# add_library(${PROJECT_NAME} SHARED
#   ${CMAKE_CURRENT_SOURCE_DIR}/src/system_state.cpp
# )

# target_include_directories(${PROJECT_NAME} PUBLIC 
#   ${CMAKE_CURRENT_SOURCE_DIR}/include
# )

# ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs state_interfaces)

install(
  DIRECTORY include
  DESTINATION include
)

ament_export_include_directories(include)
ament_package()
