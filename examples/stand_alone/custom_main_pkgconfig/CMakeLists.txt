cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(PkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
string (REPLACE ";" " " GAZEBO_CFLAGS "${GAZEBO_CFLAGS}")
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CFLAGS}")

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_executable(custom_main custom_main.cc)
target_link_libraries(custom_main ${GAZEBO_LIBRARIES})

if(${CMAKE_VERSION} VERSION_LESS "3.13.0") 
  link_directories(${Boost_LIBRARY_DIRS})
else()
  target_link_directories(custom_main PUBLIC ${Boost_LIBRARY_DIRS})
endif()
