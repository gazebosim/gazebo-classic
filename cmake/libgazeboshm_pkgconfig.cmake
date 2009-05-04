prefix=@CMAKE_INSTALL_PREFIX@
libdir=${prefix}/lib
includedir=${prefix}/include

Name: libgazeboshm
Description: Shared memory interface to Gazebo
Version: @GAZEBO_VERSION@
Requires:
Libs: -L${libdir} -lgazeboshm
CFlags: -I${includedir}
