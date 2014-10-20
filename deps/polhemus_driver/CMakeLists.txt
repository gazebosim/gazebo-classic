include (${gazebo_cmake_dir}/GazeboUtils.cmake)
include (CheckIncludeFiles)
include (CheckFunctionExists)
include (CheckLibraryExists)

include_directories(SYSTEM
  ${CMAKE_CURRENT_BINARY_DIR} 
  ${CMAKE_SOURCE_DIR}/deps/polhemus_driver/include
)

link_directories(  
)

gz_add_library(polhemus_driver SHARED src/polhemus_driver.c)
target_link_libraries(polhemus_driver usb-1.0)
gz_install_library(polhemus_driver)
gz_install_includes("polhemus_driver" include/polhemus_driver/polhemus_driver.h)

gz_add_executable(polhemus_example src/example.c)
target_link_libraries(polhemus_example polhemus_driver)
gz_install_executable(polhemus_example)
gz_add_executable(polhemus_example_continuous src/example_continuous.c)
target_link_libraries(polhemus_example_continuous polhemus_driver)
gz_install_executable(polhemus_example_continuous)
gz_add_executable(polhemus_example_multiple_sensors src/example_multiple_sensors.c)
target_link_libraries(polhemus_example_multiple_sensors polhemus_driver)
gz_install_executable(polhemus_example_multiple_sensors)
