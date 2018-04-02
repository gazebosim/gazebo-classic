cmake_minimum_required(VERSION 2.8.12)
project(example_trigger_light)

find_package(gazebo REQUIRED)

add_library(TurnOnLightPlugin SHARED TurnOnLightPlugin.cpp)
target_include_directories(TurnOnLightPlugin PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(TurnOnLightPlugin PUBLIC ${GAZEBO_LIBRARIES})
target_compile_options(TurnOnLightPlugin PUBLIC ${GAZEBO_CXX_FLAGS})
