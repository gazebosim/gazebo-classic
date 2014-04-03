add_subdirectory(opende)

if (NOT HAVE_CCD)
  add_subdirectory(libccd)
endif()

if (HAVE_LIBUDEV)
  add_subdirectory(oculus)
endif()

# add_subdirectory(ann)
# add_subdirectory(fcl)
