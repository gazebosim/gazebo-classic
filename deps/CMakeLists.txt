add_subdirectory(opende)

if (NOT HAVE_CCD)
  add_subdirectory(libccd)
endif()
# add_subdirectory(ann)
# add_subdirectory(fcl)
