cmake_minimum_required(VERSION 3.14)
project(global_lvba)

set(CMAKE_BUILD_TYPE "Release")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native -mtune=native -funroll-loops")

add_definitions(-DROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")

include(ProcessorCount)
ProcessorCount(N)
message(STATUS "Processor count: ${N}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(OpenMP REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(GLUT REQUIRED)
find_package(Sophus REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SQLITE3 REQUIRED sqlite3)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${SQLITE3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/SiftGPU/src/SiftGPU
)

set(SIFTGPU_LIBS
  ${CMAKE_CURRENT_SOURCE_DIR}/src/SiftGPU/build/src/SiftGPU/libsiftgpu.a
)

# Single executable to avoid multiple-definition issues from BALM header-defined functions
add_executable(lidar_visual_ba
  src/main.cpp
  src/lvba_system.cpp
  src/dataset_io.cpp
)

target_link_libraries(lidar_visual_ba
  Ceres::ceres
  Eigen3::Eigen
  Sophus::Sophus
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${SIFTGPU_LIBS}
  ${SQLITE3_LIBRARIES}
  GLEW::GLEW
  GLUT::GLUT
  OpenGL::GL
)
ament_target_dependencies(lidar_visual_ba rclcpp sensor_msgs pcl_conversions)

install(TARGETS
  lidar_visual_ba
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
