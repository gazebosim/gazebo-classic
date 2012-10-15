#-------------------------------------------------------------------
# This file is part of the CMake build system for SKYX
#
# The contents of this file are placed in the public domain. Feel
# free to make use of it in any way you like.
#-------------------------------------------------------------------

# Configure settings and install targets

if (WIN32)
  set(SKYX_RELEASE_PATH "/Release")
  set(SKYX_RELWDBG_PATH "/RelWithDebInfo")
  set(SKYX_MINSIZE_PATH "/MinSizeRel")
  set(SKYX_DEBUG_PATH "/Debug")
  set(SKYX_LIB_RELEASE_PATH "/Release")
  set(SKYX_LIB_RELWDBG_PATH "/RelWithDebInfo")
  set(SKYX_LIB_MINSIZE_PATH "/MinSizeRel")
  set(SKYX_LIB_DEBUG_PATH "/Debug")
  set(SKYX_PLUGIN_PATH "/opt")
  set(SKYX_SAMPLE_PATH "/opt/samples")
elseif (UNIX)
  set(SKYX_RELEASE_PATH "")
  set(SKYX_RELWDBG_PATH "")
  set(SKYX_MINSIZE_PATH "")
  set(SKYX_DEBUG_PATH "/debug")
  set(SKYX_LIB_RELEASE_PATH "")
  set(SKYX_LIB_RELWDBG_PATH "")
  set(SKYX_LIB_MINSIZE_PATH "")
  set(SKYX_LIB_DEBUG_PATH "")
  set(SKYX_PLUGIN_PATH "/SKYX")
  set(SKYX_SAMPLE_PATH "/SKYX/Samples")
endif ()

# create vcproj.user file for Visual Studio to set debug working directory
function(skyx_create_vcproj_userfile TARGETNAME)
  if (MSVC)
    configure_file(
      ${SKYX_TEMPLATES_DIR}/VisualStudioUserFile.vcproj.user.in
      ${CMAKE_CURRENT_BINARY_DIR}/${TARGETNAME}.vcproj.user
      @ONLY
    )
  endif ()
endfunction(skyx_create_vcproj_userfile)

# install targets according to current build type
function(skyx_install_target TARGETNAME SUFFIX)
  install(TARGETS ${TARGETNAME}
    BUNDLE DESTINATION "bin${SKYX_RELEASE_PATH}" CONFIGURATIONS Release None ""
    RUNTIME DESTINATION "bin${SKYX_RELEASE_PATH}" CONFIGURATIONS Release None ""
    LIBRARY DESTINATION "lib${SKYX_LIB_RELEASE_PATH}${SUFFIX}" CONFIGURATIONS Release None ""
    ARCHIVE DESTINATION "lib${SKYX_LIB_RELEASE_PATH}${SUFFIX}" CONFIGURATIONS Release None ""
    FRAMEWORK DESTINATION "bin${SKYX_RELEASE_PATH}" CONFIGURATIONS Release None ""
  )
  install(TARGETS ${TARGETNAME}
    BUNDLE DESTINATION "bin${SKYX_RELWDBG_PATH}" CONFIGURATIONS RelWithDebInfo
    RUNTIME DESTINATION "bin${SKYX_RELWDBG_PATH}" CONFIGURATIONS RelWithDebInfo
    LIBRARY DESTINATION "lib${SKYX_LIB_RELWDBG_PATH}${SUFFIX}" CONFIGURATIONS RelWithDebInfo
    ARCHIVE DESTINATION "lib${SKYX_LIB_RELWDBG_PATH}${SUFFIX}" CONFIGURATIONS RelWithDebInfo
    FRAMEWORK DESTINATION "bin${SKYX_RELWDBG_PATH}" CONFIGURATIONS RelWithDebInfo
  )
  install(TARGETS ${TARGETNAME}
    BUNDLE DESTINATION "bin${SKYX_MINSIZE_PATH}" CONFIGURATIONS MinSizeRel
    RUNTIME DESTINATION "bin${SKYX_MINSIZE_PATH}" CONFIGURATIONS MinSizeRel
    LIBRARY DESTINATION "lib${SKYX_LIB_MINSIZE_PATH}${SUFFIX}" CONFIGURATIONS MinSizeRel
    ARCHIVE DESTINATION "lib${SKYX_LIB_MINSIZE_PATH}${SUFFIX}" CONFIGURATIONS MinSizeRel
    FRAMEWORK DESTINATION "bin${SKYX_MINSIZE_PATH}" CONFIGURATIONS MinSizeRel
  )
  install(TARGETS ${TARGETNAME}
    BUNDLE DESTINATION "bin${SKYX_DEBUG_PATH}" CONFIGURATIONS Debug
    RUNTIME DESTINATION "bin${SKYX_DEBUG_PATH}" CONFIGURATIONS Debug
    LIBRARY DESTINATION "lib${SKYX_LIB_DEBUG_PATH}${SUFFIX}" CONFIGURATIONS Debug
    ARCHIVE DESTINATION "lib${SKYX_LIB_DEBUG_PATH}${SUFFIX}" CONFIGURATIONS Debug
    FRAMEWORK DESTINATION "bin${SKYX_DEBUG_PATH}" CONFIGURATIONS Debug
  )
endfunction(skyx_install_target)

# setup common target settings
function(skyx_config_common TARGETNAME)
  set_target_properties(${TARGETNAME} PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${SKYX_BINARY_DIR}/lib
    LIBRARY_OUTPUT_DIRECTORY ${SKYX_BINARY_DIR}/lib
    RUNTIME_OUTPUT_DIRECTORY ${SKYX_BINARY_DIR}/bin
  )
  skyx_create_vcproj_userfile(${TARGETNAME})
endfunction(skyx_config_common)

# setup library build
function(skyx_config_lib LIBNAME)
  skyx_config_common(${LIBNAME})
  if(SKYX_STATIC)
    # add static prefix, if compiling static version
    set_target_properties(${LIBNAME} PROPERTIES OUTPUT_NAME ${LIBNAME}Static)
  else(SKYX_STATIC)
    # add GCC visibility flags to shared library build
    if (CMAKE_COMPILER_IS_GNUCXX)
      set_target_properties(${LIBNAME} PROPERTIES COMPILE_FLAGS "${SKYX_GCC_VISIBILITY_FLAGS}")
    endif (CMAKE_COMPILER_IS_GNUCXX)
  endif(SKYX_STATIC)
  skyx_install_target(${LIBNAME} "")

  # install debug pdb files
  if(SKYX_STATIC)
    if (SKYX_INSTALL_PDB)
      install(FILES ${SKYX_BINARY_DIR}/lib${SKYX_DEBUG_PATH}/${LIBNAME}Static_d.pdb
        DESTINATION bin${SKYX_DEBUG_PATH}
        CONFIGURATIONS Debug)
      install(FILES ${SKYX_BINARY_DIR}/lib${SKYX_RELWDBG_PATH}/${LIBNAME}Static.pdb
        DESTINATION bin${SKYX_RELWDBG_PATH}
        CONFIGURATIONS RelWithDebInfo)
    endif ()
  else(SKYX_STATIC)
    if (SKYX_INSTALL_PDB)
      install(FILES ${SKYX_BINARY_DIR}/bin${SKYX_DEBUG_PATH}/${LIBNAME}_d.pdb
        DESTINATION bin${SKYX_DEBUG_PATH}
        CONFIGURATIONS Debug)
      install(FILES ${SKYX_BINARY_DIR}/bin${SKYX_RELWDBG_PATH}/${LIBNAME}.pdb
        DESTINATION bin${SKYX_RELWDBG_PATH}
        CONFIGURATIONS RelWithDebInfo)
    endif ()
  endif(SKYX_STATIC)
endfunction(skyx_config_lib)

# setup SkyX sample build
function(skyx_config_sample_common SAMPLENAME)
  skyx_config_common(${SAMPLENAME})

  # set install RPATH for Unix systems
  if (UNIX AND SKYX_FULL_RPATH)
    set_property(TARGET ${SAMPLENAME} APPEND PROPERTY
    INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)
    set_property(TARGET ${SAMPLENAME} PROPERTY INSTALL_RPATH_USE_LINK_PATH TRUE)
  endif ()

  if(NOT SKYX_STATIC)
    # add GCC visibility flags to shared library build
    if (CMAKE_COMPILER_IS_GNUCXX)
      set_target_properties(${SAMPLENAME} PROPERTIES COMPILE_FLAGS "${SKYX_GCC_VISIBILITY_FLAGS}")
      # disable "lib" prefix on Unix
      set_target_properties(${SAMPLENAME} PROPERTIES PREFIX "")
    endif (CMAKE_COMPILER_IS_GNUCXX)
  endif(NOT SKYX_STATIC)

  skyx_install_target(${SAMPLENAME} ${SKYX_SAMPLE_PATH})
endfunction(skyx_config_sample_common)

function(skyx_config_sample_exe SAMPLENAME)
  skyx_config_sample_common(${SAMPLENAME})

  # install debug pdb files - no _d on exe
  if (SKYX_INSTALL_PDB)
    install(FILES ${SKYX_BINARY_DIR}/bin${SKYX_DEBUG_PATH}/${SAMPLENAME}.pdb
      DESTINATION bin${SKYX_DEBUG_PATH}
      CONFIGURATIONS Debug
    )
    install(FILES ${SKYX_BINARY_DIR}/bin${SKYX_RELWDBG_PATH}/${SAMPLENAME}.pdb
      DESTINATION bin${SKYX_RELWDBG_PATH}
      CONFIGURATIONS RelWithDebInfo
    )
  endif ()
endfunction(skyx_config_sample_exe)

