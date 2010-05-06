
link_directories(../libgazebo)
include_directories(../libgazebo)

set (sources gazebomodel.cc gazebobody.cc Toolbase.cc)

set_source_files_properties(${sources} PROPERTIES COMPILE_FLAGS "-ggdb -g2 -Wall")

add_executable(gazebomodel gazebomodel.cc Toolbase.cc)
add_executable(gazebobody gazebobody.cc Toolbase.cc)

target_link_libraries(gazebomodel gazebo boost_program_options yaml boost_regex)
target_link_libraries(gazebobody gazebo boost_program_options yaml boost_regex)

set_target_properties(gazebomodel PROPERTIES SKIP_BUILD_RPATH TRUE)
set_target_properties(gazebomodel PROPERTIES LINK_FLAGS "${LINK_FLAGS} ${gazebo_lflags}")
set_target_properties(gazebobody PROPERTIES SKIP_BUILD_RPATH TRUE)
set_target_properties(gazebobody PROPERTIES LINK_FLAGS "${LINK_FLAGS} ${gazebo_lflags}")

install (TARGETS gazebomodel gazebobody DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
