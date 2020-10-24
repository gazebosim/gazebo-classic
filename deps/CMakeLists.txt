add_subdirectory(opende)

if (NOT CCD_FOUND)
  add_subdirectory(libccd)
endif()
# add_subdirectory(ann)
# add_subdirectory(fcl

if (WIN32 AND NOT USE_EXTERNAL_TINY_PROCESS_LIBRARY)
  add_subdirectory(tiny-process-library)
endif()
