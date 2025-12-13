cmake_minimum_required(VERSION 3.22.1)
project(mvs_ros_driver2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  /opt/MVS/include
)

# Link directories
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  link_directories(
    /opt/MVS/lib/aarch64/
  )
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  link_directories(
    /opt/MVS/lib/64/
  )
endif()

# Create executable
add_executable(mvs_camera_node 
  src/grab_trigger.cpp
)

ament_target_dependencies(mvs_camera_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  camera_info_manager
)

target_link_libraries(mvs_camera_node
  ${OpenCV_LIBS}
  MvCameraControl
  pthread
)

# Install targets
install(TARGETS
  mvs_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()