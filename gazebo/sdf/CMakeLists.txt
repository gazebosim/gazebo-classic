add_subdirectory(interface)
add_subdirectory(1.0)
add_subdirectory(1.2)

set (headers sdf.hh sdf.h)

gz_install_includes("sdf" ${headers})
