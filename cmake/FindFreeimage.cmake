include (FindPkgConfig)
include (${gazebo_cmake_dir}/GazeboUtils.cmake)

########################################
# Find packages
if (PKG_CONFIG_FOUND)
  pkg_check_modules(freeimage freeimage>=${FREEIMAGE_VERSION})
  if (NOT freeimage_FOUND)
    message (STATUS "  freeimage.pc not found, trying freeimage_include_dir and freeimage_library_dir flags.")
  endif (NOT freeimage_FOUND)
endif (PKG_CONFIG_FOUND)

if (NOT freeimage_FOUND)
  find_path(freeimage_INCLUDE_DIRS FreeImage.h)
  if (NOT freeimage_INCLUDE_DIRS)
    message (STATUS "  Looking for FreeImage.h - not found")
    BUILD_ERROR("Missing: Unable to find FreeImage.h")
  else (NOT freeimage_INCLUDE_DIRS)
    message (STATUS "  Found ${freeimage_INCLUDE_DIRS}/FreeImage.h")
    # Check the FreeImage header for the right version
    set (testFreeImageSource ${CMAKE_CURRENT_BINARY_DIR}/CMakeTmp/test_freeimage.cc)
    file (WRITE ${testFreeImageSource} 
      "#include <FreeImage.h>\nint main () { if (FREEIMAGE_MAJOR_VERSION >= ${FREEIMAGE_MAJOR_VERSION} && FREEIMAGE_MINOR_VERSION >= ${FREEIMAGE_MINOR_VERSION}) return 1; else return 0;} \n")
    try_run(FREEIMAGE_RUNS FREEIMAGE_COMPILES ${CMAKE_CURRENT_BINARY_DIR} 
                ${testFreeImageSource}
                COMPILE_DEFINITIONS "-I${freeimage_INCLUDE_DIRS}"
                COMPILE_OUTPUT_VARIABLE FREEIMAGE_COMPILE_OUTPUT)
    if (NOT FREEIMAGE_RUNS)
      message (STATUS "${FREEIMAGE_COMPILE_OUTPUT}")
      BUILD_ERROR("Invalid FreeImage Version. Requires ${FREEIMAGE_VERSION}")
    else (NOT FREEIMAGE_RUNS)
       message (STATUS "  Looking for FreeImage.h - found")
    endif (NOT FREEIMAGE_RUNS)
  endif (NOT freeimage_INCLUDE_DIRS)

  find_library(freeimage_LIBRARIES freeimage)
  if (NOT freeimage_LIBRARIES)
    message (STATUS "  Looking for libfreeimage - not found")
    BUILD_ERROR("Missing: Unable to find libfreeimage")
  else (NOT freeimage_LIBRARIES)
    message (STATUS "  Looking for libfreeimage - found")
    include_directories(${freeimage_INCLUDE_DIRS})
  endif (NOT freeimage_LIBRARIES)
endif (NOT freeimage_FOUND)
