cmake_minimum_required(VERSION 2.8.3)
project(colour_detector)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  cv_bridge
  message_generation
  std_msgs
)

add_message_files(
  FILES
  ColourDetection.msg
  ColourDetectionArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS image_transport roscpp cv_bridge message_runtime std_msgs
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_LIBRARIES}
)



add_executable(colour_detection src/colour_detection.cpp)
target_link_libraries(colour_detection
  ${catkin_LIBRARIES}
)
add_dependencies(colour_detection colour_detector_generate_messages_cpp)





