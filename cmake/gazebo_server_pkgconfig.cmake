prefix=@CMAKE_INSTALL_PREFIX@
libdir=${prefix}/lib
includedir=${prefix}/include

Name: gazebo_server
Description: Gazebo Server Library
Version: @GAZEBO_VERSION@
Requires:
Libs: -L${libdir} -lgazebo_server
CFlags: -I${includedir} -I${includedir}/gazebo 
