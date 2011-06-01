prefix=@CMAKE_INSTALL_PREFIX@
libdir=${prefix}/lib
includedir=${prefix}/include

Name: gazebo_client
Description: Gazebo Client Library
Version: @GAZEBO_VERSION@
Requires:
Libs: -L${libdir} -lgazebo_client
CFlags: -I${includedir} -I${includedir}/gazebo 
