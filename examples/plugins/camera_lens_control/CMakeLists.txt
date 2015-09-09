cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package (Qt4)
find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)

include (${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

QT4_WRAP_CPP(headers_MOC CameraLensControlExample.hh)

add_library(camera_lens_control_example SHARED ${headers_MOC}
  CameraLensControlExample.cc)
target_link_libraries(camera_lens_control_example ${GAZEBO_LIBRARIES}
    ${QT_LIBRARIES} Qt4::QtUiTools
    ${PROTOBUF_LIBRARIES})