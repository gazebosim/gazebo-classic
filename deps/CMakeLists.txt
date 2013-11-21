add_subdirectory(libccd)
add_subdirectory(opende)

if (HAVE_LIBUDEV)
  add_subdirectory(oculus)
endif()

# add_subdirectory(ann)
# add_subdirectory(fcl)
