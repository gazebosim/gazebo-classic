
link_directories(../libgazebo)
include_directories(../libgazebo)

set (sources gazebomodel.cc)

set_source_files_properties(${sources} PROPERTIES COMPILE_FLAGS "-ggdb -g2 -Wall")

add_executable(gazebomodel gazebomodel.cc)

target_link_libraries(gazebomodel gazebo boost_program_options yaml)
