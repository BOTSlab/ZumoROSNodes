cmake_minimum_required(VERSION 2.8.3)
project(keyboard_listener)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)


include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)



add_executable(keyboardListener src/keyboardListener.cpp)
target_link_libraries(keyboardListener
  ${catkin_LIBRARIES}
)





