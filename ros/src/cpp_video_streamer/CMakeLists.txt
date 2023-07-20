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
find_package(OpenCV COMPONENTS core calib3d)

if( OpenCV_FOUND )
  message("-- OpenCV found:")
  message("   -- version  : " ${OpenCV_VERSION})
  message("   -- include  : " ${OpenCV_INCLUDE_DIRS})
  message("   -- libraries: " ${OpenCV_LIBRARIES})
  include_directories(${OpenCV_INCLUDE_DIRS})
  link_directories(${OpenCV_LIBRARY_DIRS})
  add_definitions(${OpenCV_DEFINITIONS})
else()
	message("-- didn't find OpenCV on system, disabling OpenCV")
endif()
	


ament_package()
