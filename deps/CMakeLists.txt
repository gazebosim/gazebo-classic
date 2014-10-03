add_subdirectory(opende)

if (NOT CCD_FOUND)
  add_subdirectory(libccd)
endif()
# add_subdirectory(ann)
# add_subdirectory(fcl)
