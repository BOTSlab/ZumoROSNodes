cmake_minimum_required(VERSION 2.8.3)
project(pixy_node)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	sensor_msgs
	message_generation
	roscpp
	tf
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread system chrono)


add_subdirectory(libpixyusb)
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${libpixyusb_SOURCE_DIR}/cmake" )
find_package ( libusb-1.0 REQUIRED )

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   PixyData.msg
   PixyBlock.msg
   Servo.msg
)


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   sensor_msgs
)

catkin_package(
  INCLUDE_DIRS libpixyusb/include libpixyusb/common
  LIBRARIES pixyusb
  CATKIN_DEPENDS message_runtime sensor_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include libpixyusb/include libpixyusb/common  ${LIBUSB_1_INCLUDE_DIRS})

## Declare a cpp library
# add_library(pixy_ros
#   src/${PROJECT_NAME}/pixy_ros.cpp
# )

## Declare a cpp executable
add_executable(pixy_node src/pixy_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(pixy_node
  ${catkin_LIBRARIES}
  pixyusb
  ${Boost_LIBRARIES}
  ${LIBUSB_1_LIBRARY}
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(pixy_node $(catkin_EXPORTED_TARGETS))

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS pixy_node pixy_node_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pixy_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
