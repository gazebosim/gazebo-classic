@REM Meant to be run from inside build/
@set build_type=Release
@if not "%1"=="" set build_type=%1
@echo Configuring for build type %build_type%

@REM Need absolute paths in order to run anything inside build/
@echo - script path is %~dp0
@set root_path=%~dp0
@pushd %~dp0\..
@set deps_path=%CD%
@popd
@echo - dependencies path is %deps_path%

@set PATH=%deps_path%\boost_1_56_0\lib64-msvc-12.0;%deps_path%\FreeImage-vc12-x64-release-debug\x64\%build_type%\DLL;%root_path%\deps\opende;%deps_path%\ogre_src_v1-8-1-vc12-x64-release-debug\build\install\%build_type%\bin\%build_type%;%deps_path%\sdformat\build\install\%build_type%\lib;%deps_path%\tbb43_20141023oss\bin\intel64\vc12;%root_path%\deps\libccd;%root_path%\deps\opende\OPCODE;%root_path%\deps\opende\ou;%deps_path%\zlib-1.2.8-vc12-x64-release-debug\contrib\vstudio\vc11\x64\ZlibDll%build_type%;%PATH%
