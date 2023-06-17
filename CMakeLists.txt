cmake_minimum_required(VERSION 3.0.2)
project(ultra_sonic_radar_detector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_filters
  pcl_conversions
  pcl_ros
  std_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ultra_sonic_radar_detector
  CATKIN_DEPENDS roscpp message_filters pcl_conversions pcl_ros std_msgs sensor_msgs tf2 tf2_geometry_msgs tf2_ros geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# # Declare a C++ library
add_library(${PROJECT_NAME}
  src/ultra_sonic_radar_detector.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node
  src/ultra_sonic_radar_detector_main.cpp
)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

# catkin_install_python(PROGRAMS
# DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
