@REM Meant to be run from inside build/
@set build_type=Release
@if not "%1"=="" set build_type=%1
@echo Configuring for build type %build_type%
@set PATH=%cd%\..\..\boost_1_56_0\lib64-msvc-12.0;%cd%\..\..\FreeImage-vc12-x64-release-debug\x64\%build_type%\DLL;%cd%\deps\opende;%cd%\..\..\ogre_src_v1-8-1-vc12-x64-release-debug\build\install\%build_type%\bin\%build_type%;%cd%\..\..\sdformat\install\build\%build_type%\lib;%cd%\..\..\tbb43_20141023oss\bin\intel64\vc12;%cd%\deps\libccd;%cd%\deps\opende\OPCODE;%cd%\deps\opende\ou;%PATH%
