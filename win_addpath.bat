@REM Add paths to Windows. Run this script prior to running gzserver.exe
@REM and gzclient.ext on a Windows machine.
@REM Meant to be run from inside build/
@set build_type=Release
@if not "%1"=="" set build_type=%1
@echo Configuring for build type %build_type%

@REM Need absolute paths in order to run anything inside build/
@set gz_root_path=%~dp0
@set gz_build_path=%gz_root_path%\build
@pushd %~dp0\..
@set deps_path=%CD%
@popd
@echo - script path is %gz_root_path%
@echo - dependencies path is %deps_path%

@set HOME=%HOMEDRIVE%%HOMEPATH%

@set GAZEBO_MODEL_PATH=%gz_root_path%\models
@set GAZEBO_PLUGIN_PATH=%gz_build_path%\plugins
@set GAZEBO_RESOURCE_PATH=%gz_root_path%
@set OGRE_RESOURCE_PATH=%deps_path%\ogre-sdk-1.10.12-vc15-x64\bin
@REM usernames with spaces cause problems, so we set the partition manually
@set IGN_PARTITION=gazebo
@set GAZEBO_GUI_INI_FILE=%HOME%/.gazebo/gui.ini
@REM This works around the need to deploy the gzclient QT app
@set QT_QPA_PLATFORM_PLUGIN_PATH=%deps_path%\qt-opensource-windows-x86-msvc2015_64-5.7.0\plugins

@set PATH=%deps_path%\boost_1_67_0\lib64-msvc-14.1;^
%deps_path%\FreeImage\Dist\x64;^
%gz_build_path%\deps\opende;^
%deps_path%\ogre-sdk-1.10.12-vc15-x64\bin;^
%deps_path%\sdformat\build\install\%build_type%\lib;^
%deps_path%\ign-math\build\install\%build_type%\bin;^
%deps_path%\ign-msgs\build\install\%build_type%\bin;^
%deps_path%\ign-transport\build\install\%build_type%\bin;^
%deps_path%\tbb43_20141023oss\bin\intel64\vc12;^
%gz_build_path%\deps\libccd;^
%gz_build_path%\deps\opende\GIMPACT;^
%gz_build_path%\deps\opende\OPCODE;^
%gz_build_path%\deps\opende\ou;^
%deps_path%\libzip-1.4.0_zlip-1.2.11_vc15-x64-dll-MD\bin;^
%deps_path%\libzmq-4.2.3_cppzmq-4.2.2_vc15-x64-dll-MD\bin;^
%deps_path%\qt-opensource-windows-x86-msvc2015_64-5.7.0\bin;^
%deps_path%\dlfcn-win32-vc15-x64-dll-MD\bin;^
%deps_path%\curl-7.57.0-vc15-x64-dll-MD\bin;^
%deps_path%\protobuf-3.4.1-vc15-x64-dll-MD\bin;^
%deps_path%\qwt_6.1.2~osrf_qt5\%build_type%\qwt-6.1.2-vc12-x64;^
%gz_build_path%\plugins;^
%gz_build_path%\tools;^
%gz_build_path%\gazebo;^
%gz_build_path%\gazebo\common;^
%gz_build_path%\gazebo\gui;^
%gz_build_path%\gazebo\msgs;^
%gz_build_path%\gazebo\physics;^
%gz_build_path%\gazebo\rendering;^
%gz_build_path%\gazebo\sensors;^
%gz_build_path%\gazebo\transport;^
%gz_build_path%\gazebo\util;^
%PATH%
