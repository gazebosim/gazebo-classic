prefix=@CMAKE_INSTALL_PREFIX@
libdir=${prefix}/lib
includedir=${prefix}/include

Name: gazeboserver
Description: Gazebo server
Version: @GAZEBO_VERSION@
Requires:
Libs: -L${libdir} -lgazebo_server -lgazebo @gazeboserver_ldflags@ 
CFlags: -I${includedir}
