cmake_minimum_required(VERSION 2.8.3)
project(apriltags_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  apriltags
  image_transport
  roscpp
  tf
  cv_bridge
  message_generation
  std_msgs
  sensor_msgs
)

add_message_files(
  FILES
  AprilTagDetection.msg
  AprilTagDetectionArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  LIBRARIES apriltag_detector
  CATKIN_DEPENDS apriltags image_transport roscpp tf cv_bridge message_runtime std_msgs sensor_msgs
  DEPENDS Eigen
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)



add_executable(apriltag_detector src/apriltag_detector.cpp)
target_link_libraries(apriltag_detector
  ${catkin_LIBRARIES}
)
add_dependencies(apriltag_detector apriltag_detector_generate_messages_cpp)




#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS apriltag_detector
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/apriltag_detector
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
