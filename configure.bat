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

@set BOOST_PATH=%cd%\..\..\install\boost_1_67_0
@set BOOST_LIBRARY_DIR=%BOOST_PATH%\lib64-msvc-14.1

set CURL_LIBRARY_NAME=libcurl_a
@if "%build_type%"=="Debug" set CURL_LIBRARY_NAME=libcurl_a_debug

@set FREEIMAGE_PATH=%cd%\..\..\install\freeimage
@set FREEIMAGE_LIBRARY_DIR=%FREEIMAGE_PATH%\dist\x64\
@set FREEIMAGE_INCLUDE_DIR=%FREEIMAGE_PATH%\dist\x64\

@set SDFORMAT_PATH=%cd%\..\..\sdformat\build\install\%build_type%
@set IGNITION-MATH_PATH=%cd%\..\..\ign-math\build\install\%build_type%
@set IGNITION-MSGS_PATH=%cd%\..\..\ign-msgs\build\install\%build_type%
@set IGNITION-TRANSPORT_PATH=%cd%\..\..\ign-transport\build\install\%build_type%
@set IGNITION-TRANSPORT_CMAKE_PREFIX_PATH=%IGNITION-TRANSPORT_PATH%\lib\cmake\ignition-transport3

@set TBB_PATH=%cd%\..\..\install\tbb43_20141023oss
@set TBB_LIBRARY_DIR=%TBB_PATH%\lib\intel64\vc12
@set TBB_INCLUDEDIR=%TBB_PATH%\include

@set QWT_PATH=%cd%\..\..\install\qwt_6.1.2~osrf_qt5
@set QWT_LIBRARY_DIR=%QWT_PATH%\%build_type%\qwt-6.1.2-vc12-x64
@set QWT_INCLUDEDIR=%QWT_PATH%\include

@set OGRE_VERSION=1.10.12
@set OGRE_PATH=%cd%\..\..\install\ogre-sdk-1.10.12-vc15-x64
@set OGRE_INCLUDE_DIR=%OGRE_PATH%\include;%OGRE_PATH%\include\OGRE;%OGRE_PATH%\include\OGRE\RTShaderSystem;%OGRE_PATH%\include\OGRE\Terrain;%OGRE_PATH%\include\OGRE\Paging
@set OGRE_LIBRARY_DIR=%OGRE_PATH%\lib
@set OGRE_PLUGIN_DIR=%OGRE_PATH%\bin
set OGRE_LIB_SUFFIX=.lib
@if "%build_type%"=="Debug" set OGRE_LIB_SUFFIX=_d.lib
@set OGRE_LIBS=%OGRE_LIBRARY_DIR%\OgreMain%OGRE_LIB_SUFFIX%;%OGRE_LIBRARY_DIR%\OgreRTShaderSystem%OGRE_LIB_SUFFIX%;%OGRE_LIBRARY_DIR%\OgreTerrain%OGRE_LIB_SUFFIX%;%OGRE_LIBRARY_DIR%\OgrePaging%OGRE_LIB_SUFFIX%;%OGRE_LIBRARY_DIR%\OgreOverlay%OGRE_LIB_SUFFIX%

@set QT5_PATH=%cd%\..\..\install\qt-opensource-windows-x86-msvc2015_64-5.7.0
@set QT5_BIN_DIR=%QT5_PATH%\bin

@set INCLUDE=%WORKSPACE_INSTALL_DIR%\include;%FREEIMAGE_INCLUDE_DIR%;%TBB_INCLUDEDIR%;%INCLUDE%
@set LIB=%WORKSPACE_INSTALL_DIR%\lib;%BOOST_LIBRARY_DIR%;%FREEIMAGE_LIBRARY_DIR%;%TBB_LIBRARY_DIR%;%LIB%

@set PATH=%QT5_BIN_DIR%;%PATH%

:: Use legacy install location if unset
@if "%WORKSPACE_INSTALL_DIR%"=="" set WORKSPACE_INSTALL_DIR="install\%build_type%"

cmake -Wno-dev -G "NMake Makefiles JOM"^
    -DCMAKE_PREFIX_PATH="%SDFORMAT_PATH%;%IGNITION-MATH_PATH%;%IGNITION-MSGS_PATH%;%IGNITION-TRANSPORT_CMAKE_PREFIX_PATH%;%WORKSPACE_INSTALL_DIR%"^
    -DUSE_EXTERNAL_TINYXML:BOOL=False^
    -DUSE_EXTERNAL_TINYXML2:BOOL=False^
    -DFREEIMAGE_RUNS=1^
    -DBOOST_ROOT:STRING="%BOOST_PATH%"^
    -DBOOST_LIBRARYDIR:STRING="%BOOST_LIBRARY_DIR%"^
    -DOGRE_FOUND=1^
    -DOGRE-RTShaderSystem_FOUND=1^
    -DOGRE-Terrain_FOUND=1^
    -DOGRE-Overlay_FOUND=1^
    -DOGRE_VERSION=%OGRE_VERSION%^
    -DOGRE_PLUGINDIR="%OGRE_PLUGIN_DIR%"^
    -DOGRE_INCLUDE_DIRS="%OGRE_INCLUDE_DIR%"^
    -DOGRE_LIBRARIES="%OGRE_LIBS%"^
    -DQWT_WIN_INCLUDE_DIR="%QWT_INCLUDEDIR%"^
    -DQWT_WIN_LIBRARY_DIR="%QWT_LIBRARY_DIR%"^
    -DCURL_FOUND=1^
    -DCURL_LIBRARIES="%CURL_LIBRARY_NAME%"^
    -DTBB_FOUND=1^
    -DTBB_INCLUDEDIR="%TBB_INCLUDEDIR%"^
    -DTBB_LIBRARY_DIR="%TBB_LIBRARY_DIR%"^
    -DCMAKE_INSTALL_PREFIX="%WORKSPACE_INSTALL_DIR%"^
    -DCMAKE_BUILD_TYPE="%build_type%"^
    ..
