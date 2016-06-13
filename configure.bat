:: This file is a helper for gazebo configuration (cmake) on Windows
::
:: It is designed to work with a workspace detailed in the INSTALL_WIN32.md
:: file. Please follow the instructions there. The workspace layout should be:
::
:: gz-ws/
::   sdformat/
::   ...     # all dependencies detailed in the INSTALL file
::   #dep1/
::   #dep2/
::   ...
::   gazebo/
::
:: Usage: cd gz-ws/gazebo/build && ../configure <Release|Debug>
::
:: ToDo: Determine why cmake findpackage does not work for protobuf and why there are no cmake package files in the gazebo specified protobuf download.
:: ToDo: Determine why cmake findpackage is used for SDFormat with WIN32 but findpackage is excluded for ignition-math2 and ignition-transport1.

@set build_type=Release
@if not "%1"=="" set build_type=%1
@echo Configuring for build type %build_type%

@set generation_type="NMake Makefiles"
@if not %2.==. set generation_type=%2
@echo Generate type %generation_type%

@set OGRE_VERSION=1.9.0
@set OGRE_PATH=F:\Ogre3D\OGRE-SDK-1.9.0-vc120-x64-12.03.2016
@set BOOST_PATH=F:\boost\boost_1_56_0
@set GAZEBO_SOURCE_DIR=f:\gazebo-build
@set PROTOBUF_PATH=%GAZEBO_SOURCE_DIR%\protobuf-2.6.0-win64-vc12
@set CURL_PATH=%GAZEBO_SOURCE_DIR%\libcurl-vc12-x64-release-debug-static-ipv6-sspi-winssl\%build_type%
@set FREEIMAGE_PATH=%GAZEBO_SOURCE_DIR%\FreeImage-vc12-x64-release-debug
@set SDFORMAT_PATH=%GAZEBO_SOURCE_DIR%\sdformat\build_%build_type%\install\%build_type%
@set IGN_MATH_PATH=%GAZEBO_SOURCE_DIR%\ign-math\build\install\%build_type%\install\%build_type%
@set IGN-TRANSPORT_PATH=%GAZEBO_SOURCE_DIR%\ign-transport\build_%build_type%\install\%build_type%
@set DLFCN_WIN32_PATH=%GAZEBO_SOURCE_DIR%\dlfcn-win32-vc12-x64-release-debug\build\install\%build_type%
@set ZEROMQ_PATH=F:\ZeroMQ4.0.4
@set CPPZMQ_PATH=%GAZEBO_SOURCE_DIR%\cppzmq
@set TBB_PATH=%GAZEBO_SOURCE_DIR%\tbb43_20141023oss
@set QT4_PATH=F:\Qt\4.8.6\x64\msvc2013

@set BOOST_LIBRARY_DIR=%BOOST_PATH%\lib64-msvc-12.0

@set CURL_INCLUDE_DIR=%CURL_PATH%\include
@set CURL_LIBRARY_DIR=%CURL_PATH%\lib
@set CURL_LIBRARY_NAME=libcurl_a
@if "%build_type%"=="Debug" set CURL_LIBRARY_NAME=libcurl_a_debug

@set FREEIMAGE_LIBRARY_DIR=%FREEIMAGE_PATH%\x64\%build_type%\DLL
@set FREEIMAGE_INCLUDE_DIR=%FREEIMAGE_PATH%\Source

@set TBB_LIBRARY_DIR=%TBB_PATH%\lib\intel64\vc12
@set TBB_INCLUDEDIR=%TBB_PATH%\include

@set OGRE_INCLUDE_DIR=%OGRE_PATH%\include;%OGRE_PATH%\include\OGRE;%OGRE_PATH%\include\OGRE\RTShaderSystem;%OGRE_PATH%\include\OGRE\Terrain;%OGRE_PATH%\include\OGRE\Paging
@set OGRE_LIBRARY_DIR=%OGRE_PATH%\lib\%build_type%
@set OGRE_PLUGIN_DIR=%OGRE_LIBRARY_DIR%\opt
@set OGRE_LIBS=%OGRE_LIBRARY_DIR%\OgreMain%OGRE_LIB_SUFFIX%;%OGRE_LIBRARY_DIR%\OgreOverlay%OGRE_LIB_SUFFIX%;%OGRE_LIBRARY_DIR%\OgreRTShaderSystem%OGRE_LIB_SUFFIX%;%OGRE_LIBRARY_DIR%\OgreTerrain%OGRE_LIB_SUFFIX%;%OGRE_LIBRARY_DIR%\OgrePaging%OGRE_LIB_SUFFIX%
@set OGRE_LIB_SUFFIX=.lib
@if "%build_type%"=="Debug" set OGRE_LIB_SUFFIX=_d.lib

@set DLFCN_WIN32_LIBRARY_DIR=%DLFCN_WIN32_PATH%\lib
@set DLFCN_WIN32_INCLUDE_DIR=%DLFCN_WIN32_PATH%\include

@set QT4_BIN_DIR=%QT4_PATH%\bin

@set ZERO_MQ_LIB=libzmq-v120-mt-4_0_4.lib
@if "%build_type%"=="Debug" set ZERO_MQ_LIB=libzmq-v120-mt-gd-4_0_4.lib

:: Use MSVC_CXX_WARNING_FLAGS to set the warning parameter passed to the cl.exe C++ compiler. 
:: To get all warnings, which was previously the default behaviour, change to /Wall and get ready for the deluge.
@set MSVC_CXX_WARNING_FLAGS=/W1 /wd4005 /wd4068 /wd4996

@set INCLUDE=%FREEIMAGE_INCLUDE_DIR%;%TBB_INCLUDEDIR%;%DLFCN_WIN32_INCLUDE_DIR%;%INCLUDE%
@set LIB=%FREEIMAGE_LIBRARY_DIR%;%BOOST_LIBRARY_DIR%;%TBB_LIBRARY_DIR%;%DLFCN_WIN32_LIBRARY_DIR%;%LIB%
@set PATH=%QT4_BIN_DIR%;%PATH%

cmake -Wno-dev -G %generation_type%^
    -DCMAKE_PREFIX_PATH="%SDFORMAT_PATH%;%IGNITION-MATH_PATH%;%IGNITION-TRANSPORT_CMAKE_PREFIX_PATH%"^
    -DUSE_EXTERNAL_TINYXML:BOOL=False^
    -DUSE_EXTERNAL_TINYXML2:BOOL=False^
    -DFREEIMAGE_RUNS=1^
    -DPROTOBUF_FOUND=1^
    -DPROTOBUF_PROTOC_EXECUTABLE="%PROTOBUF_PATH%\vsprojects\%build_type%\protoc.exe"^
    -DPROTOBUF_PROTOC_LIBRARY="%PROTOBUF_PATH%\vsprojects\%build_type%\libprotoc.lib"^
    -DPROTOBUF_LIBRARY="%PROTOBUF_PATH%\vsprojects\%build_type%\libprotobuf.lib"^
    -DPROTOBUF_LIBRARY_DEBUG="%PROTOBUF_PATH%\vsprojects\%build_type%\libprotobuf.lib"^
    -DPROTOBUF_PROTOC_LIBRARY_DEBUG="%PROTOBUF_PATH%\vsprojects\%build_type%\libprotoc.lib"^
    -DPROTOBUF_INCLUDE_DIR="%PROTOBUF_PATH%\src"^
    -DBOOST_ROOT:STRING="%BOOST_PATH%"^
    -DBOOST_LIBRARYDIR:STRING="%BOOST_LIBRARY_DIR%"^
    -DOGRE_FOUND=1^
    -DOGRE-RTShaderSystem_FOUND=1^
    -DOGRE-Terrain_FOUND=1^
    -DOGRE_VERSION=%OGRE_VERSION%^
    -DOGRE_PLUGINDIR="%OGRE_PLUGIN_DIR%"^
    -DOGRE_INCLUDE_DIRS="%OGRE_INCLUDE_DIR%"^
    -DOGRE_LIBRARIES="%OGRE_LIBS%"^
    -DCURL_FOUND=1^
    -DCURL_INCLUDEDIR="%CURL_INCLUDE_DIR%"^
    -DCURL_LIBDIR="%CURL_LIBRARY_DIR%"^
    -DCURL_LIBRARIES="%CURL_LIBRARY_NAME%"^
    -DTBB_FOUND=1^
    -DTBB_INCLUDEDIR="%TBB_INCLUDEDIR%"^
    -DTBB_LIBRARY_DIR="%TBB_LIBRARY_DIR%"^
    -DIGNITION-MATH_INCLUDE_DIRS:STRING="%IGN_MATH_PATH%\include\ignition\math2"^
    -DIGNITION-MATH_LIBRARY_DIRS:STRING="%IGN_MATH_PATH%\lib"^
    -DIGNITION-MATH_LIBRARIES="ignition-math2"^
    -DIGNITION-TRANSPORT_INCLUDE_DIRS:STRING="%IGN-TRANSPORT_PATH%\include\ignition\transport1"^
    -DIGNITION-TRANSPORT_LIBRARY_DIRS:STRING="%IGN-TRANSPORT_PATH%\lib"^
    -DIGNITION-TRANSPORT_LIBRARIES="ignition-transport1.lib"^
    -DZERO_MQ_INCLUDE_DIRS="%ZEROMQ_PATH%\include"^
	-DZERO_MQ_LIB_DIRS="%ZEROMQ_PATH%\lib"^
    -DZERO_MQ_LIB="%ZERO_MQ_LIB%"^
    -DCPPZMQ_INCLUDE_DIRS="%CPPZMQ_PATH%"^
    -DCMAKE_INSTALL_PREFIX="install\%build_type%"^
    -DCMAKE_BUILD_TYPE="%build_type%"^
    -DENABLE_TESTS_COMPILATION:BOOL=False^
    -DMSVC_CXX_WARNING_FLAGS:STRING="%MSVC_CXX_WARNING_FLAGS%"^
    ..
