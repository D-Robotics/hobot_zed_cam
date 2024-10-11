cmake_minimum_required(VERSION 3.8)
project(hobot_zed_cam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

############################################################################
# Sources
set(SRC_VIDEO
    ${PROJECT_SOURCE_DIR}/src/videocapture.cpp
)

set(SRC_SENSORS
    ${PROJECT_SOURCE_DIR}/src/sensorcapture.cpp
)

############################################################################
# Includes
set(HEADERS_VIDEO
    # Base
    ${PROJECT_SOURCE_DIR}/include/videocapture.hpp
    
    # Defines
    ${PROJECT_SOURCE_DIR}/include/defines.hpp
    ${PROJECT_SOURCE_DIR}/include/videocapture_def.hpp
)

set(HEADERS_SENSORS
    # Base
    ${PROJECT_SOURCE_DIR}/include/sensorcapture.hpp

    # Defines
    ${PROJECT_SOURCE_DIR}/include/defines.hpp
    ${PROJECT_SOURCE_DIR}/include/sensorcapture_def.hpp
)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
)

############################################################################
# Required external libraries

list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake")

############################################################################

message("=> Sensors module available")
add_definitions(-DSENSORS_MOD_AVAILABLE)
find_package(HIDAPI REQUIRED)
message("=> libhidapi: include dir at ${HIDAPI_INCLUDE_DIRS}")
message("=> libhidapi: libraries at ${HIDAPI_LIBRARIES}")
include_directories(${HIDAPI_INCLUDE_DIRS})
set(SRC_FULL ${SRC_FULL} ${SRC_SENSORS})
set(HDR_FULL ${HDR_FULL} ${HEADERS_SENSORS})
set(DEP_LIBS
  ${DEP_LIBS}
  ${LibUSB_LIBRARIES}
  ${HIDAPI_LIBRARIES}
  pthread
)

message("=> Video module available")
add_definitions(-DVIDEO_MOD_AVAILABLE)
find_package(LIBUSB 1 REQUIRED)
message("=> libusb: include dir at ${LibUSB_INCLUDE_DIRS}")
message("=> libusb: libraries at ${LibUSB_LIBRARIES}")
include_directories(${LibUSB_INCLUDE_DIRS})
set(SRC_FULL ${SRC_FULL} ${SRC_VIDEO})
set(HDR_FULL ${HDR_FULL} ${HEADERS_VIDEO})
set(DEP_LIBS
  ${DEP_LIBS}
  ${LibUSB_LIBRARIES}
)

############################################################################

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# add executable
add_executable(pub_stereo_imgs_nv12 src/pub_stereo_imgs_nv12.cpp ${SRC_FULL})
target_include_directories(pub_stereo_imgs_nv12 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pub_stereo_imgs_nv12 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(pub_stereo_imgs_nv12
  rclcpp
  sensor_msgs
  OpenCV
  cv_bridge
)
target_link_libraries(pub_stereo_imgs_nv12
    ${DEP_LIBS}
)

# install
install(TARGETS pub_stereo_imgs_nv12 DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

############################################################################

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
