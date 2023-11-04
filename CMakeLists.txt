cmake_minimum_required(VERSION 3.28)
project(steering_wheel)



set(CMAKE_CXX_STANDARD 20)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)

set (SOURCE_STEERING_WHEEL
src/controller.cc
src/markers_parser.cc
src/steering_wheel.cc)

add_executable(${PROJECT_NAME} ${SOURCE_STEERING_WHEEL})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
