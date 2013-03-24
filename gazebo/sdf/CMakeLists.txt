add_subdirectory(interface)
add_subdirectory(1.0)
add_subdirectory(1.2)
add_subdirectory(1.3)
add_subdirectory(1.4)

set (headers sdf.hh)

gz_install_includes("sdf" ${headers})
