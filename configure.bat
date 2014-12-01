@set build_type=Release
@if not "%1"=="" set build_type=%1
@echo Configuring for build type %build_type%

@set BOOST_PATH=%cd%\..\..\boost_1_56_0
@set BOOST_LIBRARY_DIR=%BOOST_PATH%\lib64-msvc-12.0

@set PROTOBUF_PATH=%cd%\..\..\protobuf-2.6.0-win64-vc12

@set CURL_PATH=%cd%\..\..\libcurl-vc12-x64-release-static-ipv6-sspi-winssl
@set CURL_INCLUDE_DIR=%CURL_PATH%\include
@set CURL_LIBRARY_DIR=%CURL_PATH%\lib

@set FREEIMAGE_PATH=%cd%\..\..\FreeImage-vc12-x64-release-debug
@set FREEIMAGE_LIBRARY_DIR=%FREEIMAGE_PATH%\x64\%build_type%
@set FREEIMAGE_INCLUDE_DIR=%FREEIMAGE_PATH%\Source

@set SDFORMAT_PATH=%cd%\..\..\sdformat\build\install\%build_type%

@set INCLUDE=%FREEIMAGE_INCLUDE_DIR%;%INCLUDE%
@set LIB=%FREEIMAGE_LIBRARY_DIR%;%BOOST_LIBRARY_DIR%;%LIB%

cmake -G "NMake Makefiles" -DCMAKE_PREFIX_PATH="%SDFORMAT_PATH%" -DFREEIMAGE_RUNS=1 -DPROTOBUF_SRC_ROOT_FOLDER="%PROTOBUF_PATH%" -DBOOST_ROOT:STRING="%BOOST_PATH%" -DBOOST_LIBRARYDIR:STRING="%BOOST_LIBRARY_DIR%" -DOGRE_VERSION=1.8.1 -DOGRE_PLUGINDIR="..\..\OgreSDK_vc10_v1-8-1\lib\%build_type%\opt" -DOGRE-RTShaderSystem_FOUND=1 -DOGRE-RTShaderSystem_INCLUDE_DIRS="..\..\OgreSDK_vc10_v1-8-1\include" -DOGRE-RTShaderSystem_LIBRARIES="..\..\OgreSDK_vc10_v1-8-1\lib\release\OgreRTShaderSystem.lib" -DCURL_FOUND=1 -DCURL_INCLUDEDIR="%CURL_INCLUDE_DIR%" -DCURL_LIBDIR="%CURL_LIBRARY_DIR%" -DCURL_LIBRARIES="libcurl_a" -DCMAKE_INSTALL_PREFIX="install\%build_type%" -DCMAKE_BUILD_TYPE="%build_type%" ..
