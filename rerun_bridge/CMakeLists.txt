cmake_minimum_required(VERSION 3.0.2)
project(rerun_bridge)

if(NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(catkin REQUIRED COMPONENTS roscpp roslib sensor_msgs nav_msgs geometry_msgs cv_bridge tf2_ros tf2_msgs)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

include(FetchContent)
FetchContent_Declare(rerun_sdk URL https://build.rerun.io/commit/359c2cd/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)

include_directories(
  include
  ${YAML_CPP_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp roslib sensor_msgs nav_msgs geometry_msgs cv_bridge tf2_ros tf2_msgs
  DEPENDS opencv yaml-cpp
)

add_library(${PROJECT_NAME} src/rerun_bridge/rerun_ros_interface.cpp)
add_executable(visualizer src/rerun_bridge/visualizer_node.cpp)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(visualizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES} rerun_sdk)
target_link_libraries(visualizer ${PROJECT_NAME} ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} rerun_sdk)

install(TARGETS visualizer DESTINATION bin)
