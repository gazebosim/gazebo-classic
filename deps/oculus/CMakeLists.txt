include (${gazebo_cmake_dir}/GazeboUtils.cmake)

include_directories(SYSTEM
  ${CMAKE_CURRENT_BINARY_DIR} 
)

set (sources 
  Src/OVR_DeviceHandle.cpp 
  Src/OVR_DeviceImpl.cpp 
  Src/OVR_JSON.cpp 
  Src/OVR_LatencyTestImpl.cpp 
  Src/OVR_Profile.cpp 
  Src/OVR_SensorFilter.cpp 
  Src/OVR_SensorFusion.cpp 
  Src/OVR_SensorImpl.cpp 
  Src/OVR_ThreadCommandQueue.cpp 
  Src/Kernel/OVR_Alg.cpp 
  Src/Kernel/OVR_Allocator.cpp 
  Src/Kernel/OVR_Atomic.cpp 
  Src/Kernel/OVR_File.cpp 
  Src/Kernel/OVR_FileFILE.cpp 
  Src/Kernel/OVR_Log.cpp 
  Src/Kernel/OVR_Math.cpp 
  Src/Kernel/OVR_RefCount.cpp 
  Src/Kernel/OVR_Std.cpp 
  Src/Kernel/OVR_String.cpp 
  Src/Kernel/OVR_String_FormatUtil.cpp 
  Src/Kernel/OVR_String_PathUtil.cpp 
  Src/Kernel/OVR_SysFile.cpp 
  Src/Kernel/OVR_System.cpp 
  Src/Kernel/OVR_Timer.cpp 
  Src/Kernel/OVR_UTF8Util.cpp 
  Src/Util/Util_LatencyTest.cpp 
  Src/Util/Util_Render_Stereo.cpp 
  Src/Kernel/OVR_ThreadsPthread.cpp 
  Src/OVR_Linux_HIDDevice.cpp 
  Src/OVR_Linux_SensorDevice.cpp 
  Src/OVR_Linux_DeviceManager.cpp 
  Src/OVR_Linux_HMDDevice.cpp
)

set (headers 
  include/OVR.h
  include/OVRVersion.h
)

gz_add_library(gazebo_ovr ${sources})
target_link_libraries(gazebo_ovr udev ${tinyxml_libraries})

gz_install_library(gazebo_ovr)
gz_install_includes("ovr" ${headers})
