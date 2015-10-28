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
@set build_type=Release
@if not "%1"=="" set build_type=%1
@echo Configuring for build type %build_type%

@set BOOST_PATH=%cd%\..\..\boost_1_56_0
@set BOOST_LIBRARY_DIR=%BOOST_PATH%\lib64-msvc-12.0

@set PROTOBUF_PATH=%cd%\..\..\protobuf-2.6.1

@set CURL_PATH=%cd%\..\..\libcurl-vc12-x64-release-debug-static-ipv6-sspi-winssl\%build_type%
@set CURL_INCLUDE_DIR=%CURL_PATH%\include
@set CURL_LIBRARY_DIR=%CURL_PATH%\lib
set CURL_LIBRARY_NAME=libcurl_a
@if "%build_type%"=="Debug" set CURL_LIBRARY_NAME=libcurl_a_debug

@set FREEIMAGE_PATH=%cd%\..\..\FreeImage\Dist
@set FREEIMAGE_LIBRARY_DIR=%FREEIMAGE_PATH%\x64
@set FREEIMAGE_INCLUDE_DIR=%FREEIMAGE_PATH%\x64

@set SDFORMAT_PATH=%cd%\..\..\sdformat\build\install\%build_type%
@set IGNITION-MATH_PATH=%cd%\..\..\ign-math\build\install\%build_type%
@set IGNITION-TRANSPORT_PATH=%cd%\..\..\ign-transport\build\install\%build_type%
@set IGNITION-TRANSPORT_CMAKE_PREFIX_PATH=%IGNITION-TRANSPORT_PATH%\lib\cmake\ignition-transport0

@set TBB_PATH=%cd%\..\..\tbb43_20141023oss
@set TBB_LIBRARY_DIR=%TBB_PATH%\lib\intel64\vc12
@set TBB_INCLUDEDIR=%TBB_PATH%\include

@set OGRE_VERSION=1.8.1
@set OGRE_PATH=%cd%\..\..\ogre_src_v1-8-1-vc12-x64-release-debug\build\install\%build_type%
@set OGRE_INCLUDE_DIR=%OGRE_PATH%\include;%OGRE_PATH%\include\OGRE;%OGRE_PATH%\include\OGRE\RTShaderSystem;%OGRE_PATH%\include\OGRE\Terrain;%OGRE_PATH%\include\OGRE\Paging
@set OGRE_LIBRARY_DIR=%OGRE_PATH%\lib\%build_type%
@set OGRE_PLUGIN_DIR=%OGRE_LIBRARY_DIR%\opt
set OGRE_LIB_SUFFIX=.lib
@if "%build_type%"=="Debug" set OGRE_LIB_SUFFIX=_d.lib
@set OGRE_LIBS=%OGRE_LIBRARY_DIR%\OgreMain%OGRE_LIB_SUFFIX%;%OGRE_LIBRARY_DIR%\OgreOverlay%OGRE_LIB_SUFFIX%;%OGRE_LIBRARY_DIR%\OgreRTShaderSystem%OGRE_LIB_SUFFIX%;%OGRE_LIBRARY_DIR%\OgreTerrain%OGRE_LIB_SUFFIX%;%OGRE_LIBRARY_DIR%\OgrePaging%OGRE_LIB_SUFFIX%
@set OGRE_LIBS=%OGRE_LIBRARY_DIR%\OgreMain%OGRE_LIB_SUFFIX%;%OGRE_LIBRARY_DIR%\OgreRTShaderSystem%OGRE_LIB_SUFFIX%;%OGRE_LIBRARY_DIR%\OgreTerrain%OGRE_LIB_SUFFIX%;%OGRE_LIBRARY_DIR%\OgrePaging%OGRE_LIB_SUFFIX%

@set DLFCN_WIN32_PATH=%cd%\..\..\dlfcn-win32-vc12-x64-release-debug\build\install\%build_type%
@set DLFCN_WIN32_LIBRARY_DIR=%DLFCN_WIN32_PATH%\lib
@set DLFCN_WIN32_INCLUDE_DIR=%DLFCN_WIN32_PATH%\include

@set QT4_PATH=C:\Qt\4.8.6\x64\msvc2013
@set QT4_BIN_DIR=%QT4_PATH%\bin

@set IGN_MATH_PATH=%cd%\..\..\ign-math\build\install\%build_type%

@set ZEROMQ_PATH=%cd%\..\..\ZeroMQ-3.2.4

@set CPPZMQ_PATH=%cd%\..\..\cppzmq

@set INCLUDE=%FREEIMAGE_INCLUDE_DIR%;%TBB_INCLUDEDIR%;%DLFCN_WIN32_INCLUDE_DIR%;%INCLUDE%
@set LIB=%FREEIMAGE_LIBRARY_DIR%;%BOOST_LIBRARY_DIR%;%TBB_LIBRARY_DIR%;%DLFCN_WIN32_LIBRARY_DIR%;%LIB%
@set PATH=%QT4_BIN_DIR%;%PATH%

cmake -G "NMake Makefiles"^
    -DCMAKE_PREFIX_PATH="%SDFORMAT_PATH%;%IGNITION-MATH_PATH%;%IGNITION-TRANSPORT_CMAKE_PREFIX_PATH%"^
    -DUSE_EXTERNAL_TINYXML:BOOL=False^
    -DUSE_EXTERNAL_TINYXML2:BOOL=False^
    -DFREEIMAGE_RUNS=1^
    -DPROTOBUF_SRC_ROOT_FOLDER="%PROTOBUF_PATH%"^
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
    -DZeroMQ_ROOT_DIR="@ZEROMQ_PATH@"^
    -DCPPZMQ_HEADER_PATH="@CPPZMQ_PATH@"^
    -DCMAKE_INSTALL_PREFIX="install\%build_type%"^
    -DCMAKE_BUILD_TYPE="%build_type%"^
    -DENABLE_TESTS_COMPILATION:BOOL=False^
    ..
