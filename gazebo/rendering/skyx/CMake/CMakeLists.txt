#-------------------------------------------------------------------
# This file is part of the CMake build system for SkyX
#
# The contents of this file are placed in the public domain. Feel
# free to make use of it in any way you like.
#-------------------------------------------------------------------

#############################################################
# Install useful CMake modules.
# These are necessary to compile the samples from the install
# directory, but can also be used for custom projects.
#############################################################

if(WIN32 OR APPLE)
	set(SKYX_CMAKE_DIR "CMake")
else()
	set(SKYX_CMAKE_DIR "${SKYX_LIB_DIRECTORY}/SKYX/cmake")
endif()

set(INST_FILES
  Utils/SkyXFindPkgMacros.cmake
  Utils/SkyXConfigTargets.cmake
  Utils/SkyXGetVersion.cmake
  Utils/SkyXMacroLogFeature.cmake
) 

if (WIN32)
  set(INST_FILES ${INST_FILES}
    Templates/VisualStudioUserFile.vcproj.user.in
  )
endif ()

install(FILES ${INST_FILES} DESTINATION ${SKYX_CMAKE_DIR})
