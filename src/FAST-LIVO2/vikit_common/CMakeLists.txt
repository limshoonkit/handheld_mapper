# vikit_common subdirectory CMakeLists.txt
# This is a subdirectory of FAST-LIVO2 and does not need project() or ament_package()

set(CMAKE_VERBOSE_MAKEFILE OFF)

# Set build flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -D_LINUX -D_REENTRANT -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
message("Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -mmmx -msse -msse2 -msse3 -mssse3")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a")
endif()

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Dependencies
# OpenCV, Eigen3, and yaml-cpp are already found in parent

# Include dirs
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

# Set Sourcefiles
file(GLOB SOURCEFILES src/*.cpp)

# Create vikit library
add_library(vikit_plugin SHARED ${SOURCEFILES})
target_link_libraries(vikit_plugin
  ${OpenCV_LIBS}
  yaml-cpp)

# Install targets
install(TARGETS vikit_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)