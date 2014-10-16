add_subdirectory(opende)
add_subdirectory(polhemus_driver)

if (NOT CCD_FOUND)
  add_subdirectory(libccd)
endif()
# add_subdirectory(ann)
# add_subdirectory(fcl)
