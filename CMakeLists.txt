cmake_minimum_required(VERSION 2.8.3)
project(camera_topic_detection)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
  std_msgs
  OpenCV REQUIRED #added
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

#add_executable(${PROJECT_NAME}
#    src/CTdet.cpp)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES camera_topic_detection
  CATKIN_DEPENDS cv_bridge roscpp sensor_msgs std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
#add_dependencies(${PROJECT_NAME} camera_topic_uwsim_generate_messages_cpp)

add_executable(camera_topic_detection src/CTdet.cpp)
target_link_libraries(camera_topic_detection ${catkin_LIBRARIES})
add_dependencies(camera_topic_detection camera_topic_uwsim_generate_messages_cpp)
