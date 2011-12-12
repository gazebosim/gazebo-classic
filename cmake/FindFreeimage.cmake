include (FindPkgConfig)
include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (freeimage_include_dir "/usr/include/" CACHE STRING "FreeImage include paths")
set (freeimage_library_dir "/usr/lib" CACHE STRING "FreeImage library paths")
set (freeimage_library "freeimage" CACHE STRING "FreeImage library")

########################################
# Find packages
if (PKG_CONFIG_FOUND)
  pkg_check_modules(FI freeimage>=${FREEIMAGE_VERSION})
  if (NOT FI_FOUND)
    message (STATUS "  freeimage.pc not found, trying freeimage_include_dir and freeimage_library_dir flags.")
  endif (NOT FI_FOUND)
endif (PKG_CONFIG_FOUND)

if (NOT FI_FOUND)
  find_path(freeimage_include_dir FreeImage.h ${freeimage_include_dir})
  if (NOT freeimage_include_dir)
    message (STATUS "  Looking for FreeImage.h - not found")
    BUILD_ERROR("Missing: Unable to find FreeImage.h")
  else (NOT freeimage_include_dir)
    # Check the FreeImage header for the right version
    set (testFreeImageSource ${CMAKE_CURRENT_BINARY_DIR}/CMakeTmp/test_freeimage.cc)
    file (WRITE ${testFreeImageSource} 
      "#include <FreeImage.h>\nint main () { if (FREEIMAGE_MAJOR_VERSION >= ${FREEIMAGE_MAJOR_VERSION} && FREEIMAGE_MINOR_VERSION >= ${FREEIMAGE_MINOR_VERSION}) return 1; else return 0;} \n")
    try_run(FREEIMAGE_RUNS FREEIMAGE_COMPILES ${CMAKE_CURRENT_BINARY_DIR} 
                ${testFreeImageSource})
    if (NOT FREEIMAGE_RUNS)
      BUILD_ERROR("Invalid FreeImage Version. Requires ${FREEIMAGE_VERSION}")
    else (NOT FREEIMAGE_RUNS)
       message (STATUS "  Looking for FreeImage.h - found")
    endif (NOT FREEIMAGE_RUNS)

  endif (NOT freeimage_include_dir)

  find_library(freeimage_library freeimage ${freeimage_library_dir})
  if (NOT freeimage_library)
    message (STATUS "  Looking for libfreeimage - not found")
    BUILD_ERROR("Missing: Unable to find libfreeimage")
  else (NOT freeimage_library)
    message (STATUS "  Looking for libfreeimage - found")
  endif (NOT freeimage_library)

else (NOT FI_FOUND)
  APPEND_TO_CACHED_LIST(gazeboserver_include_dirs 
                        ${gazeboserver_include_dirs_desc} 
                        ${FI_INCLUDE_DIRS})
  APPEND_TO_CACHED_LIST(gazeboserver_link_dirs 
                        ${gazeboserver_link_dirs_desc} 
                        ${FI_LIBRARY_DIRS})
  APPEND_TO_CACHED_LIST(gazeboserver_link_libs 
                        ${gazeboserver_link_libs_desc} 
                        ${FI_LINK_LIBS})
  APPEND_TO_CACHED_LIST(gazeboserver_link_libs 
                        ${gazeboserver_link_libs_desc} 
                        ${FI_LIBRARIES})
  APPEND_TO_CACHED_LIST(gazeboserver_link_libs 
                        ${gazeboserver_link_libs_desc} 
                        ${FI_LDFLAGS})
endif (NOT FI_FOUND)
