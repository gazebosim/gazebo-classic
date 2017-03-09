cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package (Qt4)
find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)

include (${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

QT4_WRAP_CPP(headers_MOC GUIExampleTimeWidget.hh)

add_library(gui_example_time_widget SHARED ${headers_MOC}
  GUIExampleTimeWidget.cc)
target_link_libraries(gui_example_time_widget ${GAZEBO_LIBRARIES} 
    ${OGRE_LIBRARIES} 
    ${OGRE-Terrain_LIBRARIES}
    ${QT_LIBRARIES}
    ${PROTOBUF_LIBRARIES})
