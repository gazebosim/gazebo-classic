INCLUDE (FindPkgConfig)

SET (freeimage_include_dir "/usr/include/" CACHE STRING "FreeImage include paths")
SET (freeimage_library_dir "/usr/lib" CACHE STRING "FreeImage library paths")
SET (freeimage_library "freeimage" CACHE STRING "FreeImage library")

########################################
# Find packages
IF (PKG_CONFIG_FOUND)
  pkg_check_modules(FI freeimage>=${FREEIMAGE_VERSION})
  IF (NOT FI_FOUND)
    MESSAGE (STATUS "  freeimage.pc not found, trying freeimage_include_dir and freeimage_library_dir flags.")
  ENDIF (NOT FI_FOUND)
ENDIF (PKG_CONFIG_FOUND)

IF (NOT FI_FOUND)
  FIND_PATH(freeimage_include_dir FreeImage.h ${freeimage_include_dir})
  IF (NOT freeimage_include_dir)
    MESSAGE (STATUS "  Looking for FreeImage.h - not found")
    MESSAGE (FATAL_ERROR "  Unable to find FreeImage.h")
  ELSE (NOT freeimage_include_dir)

    # Check the FreeImage header for the right version
    SET (testFreeImageSource ${CMAKE_CURRENT_BINARY_DIR}/CMakeTmp/test_freeimage.cc)
    FILE (WRITE ${testFreeImageSource} 
      "#include <FreeImage.h>\nint main () { if (FREEIMAGE_MAJOR_VERSION >= ${FREEIMAGE_MAJOR_VERSION} && FREEIMAGE_MINOR_VERSION >= ${FREEIMAGE_MINOR_VERSION}) return 1; else return 0;} \n")
    TRY_RUN(FREEIMAGE_RUNS FREEIMAGE_COMPILES ${CMAKE_CURRENT_BINARY_DIR} 
                ${testFreeImageSource})
    IF (NOT FREEIMAGE_RUNS)
      MESSAGE (FATAL_ERROR "  Invalid FreeImage Version. Requires ${FREEIMAGE_VERSION}")
    ELSE (NOT FREEIMAGE_RUNS)
       MESSAGE (STATUS "  Looking for FreeImage.h - found")
    ENDIF (NOT FREEIMAGE_RUNS)

  ENDIF (NOT freeimage_include_dir)

  FIND_LIBRARY(freeimage_library freeimage ${freeimage_library_dir})
  IF (NOT freeimage_library)
    MESSAGE (STATUS "  Looking for libfreeimage - not found")
    MESSAGE (FATAL_ERROR "  Unable to find libfreeimage")
  ELSE (NOT freeimage_library)
    MESSAGE (STATUS "  Looking for libfreeimage - found")
  ENDIF (NOT freeimage_library)

ELSE (NOT FI_FOUND)
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
ENDIF (NOT FI_FOUND)