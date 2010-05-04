
link_directories(../libgazebo)
include_directories(../libgazebo)

set (sources gazebomodel.cc gazebobody.cc Toolbase.cc)

set_source_files_properties(${sources} PROPERTIES COMPILE_FLAGS "-ggdb -g2 -Wall")

add_executable(gazebomodel gazebomodel.cc Toolbase.cc)
add_executable(gazebobody gazebobody.cc Toolbase.cc)

target_link_libraries(gazebomodel gazebo boost_program_options yaml boost_regex)
target_link_libraries(gazebobody gazebo boost_program_options yaml boost_regex)

install (TARGETS gazebomodel gazebobody DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
