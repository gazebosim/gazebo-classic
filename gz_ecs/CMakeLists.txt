cmake_minimum_required(VERSION 2.6)
project(gz_ecs)

set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories(include)

enable_testing()

add_subdirectory(ecs_core)
add_subdirectory(systems)
add_subdirectory(test)

add_executable(gz_ecs main.cc)
# TODO load System libraries in a plugin-able fashion
target_link_libraries(gz_ecs DivideAndPrintResult GazeboECSCore)
