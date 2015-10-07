cmake_minimum_required(VERSION 2.8 FATAL_ERROR)


find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_executable(custom_main custom_main.cc)
target_link_libraries(custom_main ${GAZEBO_LIBRARIES} pthread)

add_executable(custom_main_vector custom_main_vector.cc)
target_link_libraries(custom_main_vector ${GAZEBO_LIBRARIES} pthread)
