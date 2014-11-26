@set build_type=Release
@if not "%1"=="" set build_type=%1
@echo Configuring for build type %build_type%
@set INCLUDE=%cd%\..\..\FreeImage\Dist;%INCLUDE%
@set LIB=%cd%\..\..\FreeImage\Dist;%cd%\..\..\boost_1_56_0\lib64-msvc-12.0;%LIB%
cmake -G "NMake Makefiles" -DCMAKE_PREFIX_PATH="..\sdformat\build\install\%build_type%" -DFREEIMAGE_RUNS=1 -DPROTOBUF_SRC_ROOT_FOLDER="..\..\protobuf-2.6.0-win64-vc12" -DBOOST_ROOT:STRING="..\boost_1_56_0" -DBOOST_LIBRARYDIR:STRING="..\boost_1_56_0\lib64-msvc-12.0" -DOGRE_VERSION=1.8.1 -DOGRE_PLUGINDIR="..\..\OgreSDK_vc10_v1-8-1\lib\%build_type%\opt" -DOGRE-RTShaderSystem_FOUND=1 -DOGRE-RTShaderSystem_INCLUDE_DIRS="..\..\OgreSDK_vc10_v1-8-1\include" -DOGRE-RTShaderSystem_LIBRARIES="..\..\OgreSDK_vc10_v1-8-1\lib\release\OgreRTShaderSystem.lib" -DCURL_FOUND=1 -DCURL_INCLUDE_DIR="..\libcurl\include" -DCURL_LIBRARIES="..\libcurl\lib\%build_type%\curllib.lib" -DCMAKE_INSTALL_PREFIX="install/%build_type%" -DCMAKE_BUILD_TYPE="%build_type%" ..
