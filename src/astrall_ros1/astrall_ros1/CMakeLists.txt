cmake_minimum_required(VERSION 3.5)
project(astrall_ros1)

add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
)

catkin_package()

get_filename_component(ASTRALL_SDK_AUTO_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(ASTRALL_SDK_ROOT "${ASTRALL_SDK_AUTO_ROOT}" CACHE PATH
    "Path to astrall_sdk_1_0_5")

# Older builds cached the package parent (src/astrall_ros1) as the SDK root.
# Recover automatically when this package is inside the SDK catkin workspace.
if(NOT EXISTS "${ASTRALL_SDK_ROOT}/sdk/include/interface.h" AND
   EXISTS "${ASTRALL_SDK_AUTO_ROOT}/sdk/include/interface.h")
  message(WARNING
    "ASTRALL_SDK_ROOT='${ASTRALL_SDK_ROOT}' is invalid; using detected SDK root "
    "'${ASTRALL_SDK_AUTO_ROOT}'")
  set(ASTRALL_SDK_ROOT "${ASTRALL_SDK_AUTO_ROOT}" CACHE PATH
      "Path to astrall_sdk_1_0_5" FORCE)
endif()
set(ASTRALL_SDK_INCLUDE_DIR "${ASTRALL_SDK_ROOT}/sdk/include")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(ASTRALL_SDK_LIB_DIR "${ASTRALL_SDK_ROOT}/sdk/arm64")
else()
  set(ASTRALL_SDK_LIB_DIR "${ASTRALL_SDK_ROOT}/sdk/x86-64")
endif()

set(ASTRALL_SDK_LIB "${ASTRALL_SDK_LIB_DIR}/libASTRALL_SDK.so")

if(NOT EXISTS "${ASTRALL_SDK_INCLUDE_DIR}/interface.h")
  message(FATAL_ERROR "Cannot find Astrall SDK header: ${ASTRALL_SDK_INCLUDE_DIR}/interface.h")
endif()
if(NOT EXISTS "${ASTRALL_SDK_LIB}")
  message(FATAL_ERROR "Cannot find Astrall SDK library: ${ASTRALL_SDK_LIB}")
endif()

add_library(astrall_sdk SHARED IMPORTED GLOBAL)
set_target_properties(astrall_sdk PROPERTIES
  IMPORTED_LOCATION "${ASTRALL_SDK_LIB}"
  INTERFACE_INCLUDE_DIRECTORIES "${ASTRALL_SDK_INCLUDE_DIR}"
)

add_executable(astrall_driver_node src/astrall_driver_node.cpp)
target_include_directories(astrall_driver_node PRIVATE
  ${catkin_INCLUDE_DIRS}
  "${ASTRALL_SDK_INCLUDE_DIR}"
)
target_link_libraries(astrall_driver_node astrall_sdk ${catkin_LIBRARIES})
set_target_properties(astrall_driver_node PROPERTIES
  BUILD_RPATH "${ASTRALL_SDK_LIB_DIR}"
  INSTALL_RPATH "$ORIGIN/.."
)

add_executable(astrall_h264_camera_node src/astrall_h264_camera_node.cpp)
target_include_directories(astrall_h264_camera_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(astrall_h264_camera_node ${catkin_LIBRARIES})

add_executable(astrall_keyboard_teleop_node src/astrall_keyboard_teleop_node.cpp)
target_include_directories(astrall_keyboard_teleop_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(astrall_keyboard_teleop_node ${catkin_LIBRARIES})

install(TARGETS
  astrall_driver_node
  astrall_h264_camera_node
  astrall_keyboard_teleop_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY "${ASTRALL_SDK_LIB_DIR}/"
  DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION}
  FILES_MATCHING PATTERN "libASTRALL_SDK.so*"
)
