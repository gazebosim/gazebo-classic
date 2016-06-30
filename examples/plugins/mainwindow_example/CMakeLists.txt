cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package (Qt5Core)
set (CMAKE_AUTOMOC ON)

add_definitions(${QT_DEFINITIONS})

find_package(gazebo REQUIRED)
include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(main_window_example_widget SHARED ${headers_MOC}
  MainWindowExampleWidget.cc)
target_link_libraries(main_window_example_widget ${GAZEBO_LIBRARIES} 
    ${QT_LIBRARIES}
)
