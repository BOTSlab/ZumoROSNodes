cmake_minimum_required(VERSION 2.8.3)
project(main_node)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  ##message_generation
  std_msgs
)

##add_message_files(
#  FILES
#  ColourDetection.msg
#  ColourDetectionArray.msg
#)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

catkin_package(
  #CATKIN_DEPENDS roscpp cv_bridge message_runtime std_msgs
  CATKIN_DEPENDS roscpp std_msgs
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)



add_executable(defaultBehaviour src/defaultBehaviour.cpp)
target_link_libraries(defaultBehaviour
  ${catkin_LIBRARIES}
)
add_dependencies(defaultBehaviour ${catkin_EXPORTED_TARGETS})
#add_dependencies(defaultBehaviour defaultBehaviour_generate_messages_cpp)




