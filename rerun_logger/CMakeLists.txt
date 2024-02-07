cmake_minimum_required(VERSION 3.0.2)
project(rerun_logger)

if(NOT DEFINED CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs nav_msgs cv_bridge)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

include(FetchContent)
FetchContent_Declare(rerun_sdk URL https://build.rerun.io/commit/79d8021/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)

include_directories(
  ${YAML_CPP_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/rerun_logger/rerun_logger_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES} rerun_sdk)
