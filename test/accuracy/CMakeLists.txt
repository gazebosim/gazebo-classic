include_directories (
  ${CMAKE_SOURCE_DIR}/deps/opende/include
  ${ODE_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIR}
  ${OGRE_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIR}
)

link_directories(
  ${ogre_library_dirs}
  ${Boost_LIBRARY_DIRS}
  ${ODE_LIBRARY_DIRS}
)

if(HAVE_BULLET)
  include_directories ( ${BULLET_INCLUDE_DIRS} )
  link_directories ( ${BULLET_LIBRARY_DIRS} )
endif()
if(HAVE_DART)
  include_directories ( ${DARTCore_INCLUDE_DIRS} )
  link_directories ( ${DARTCore_LIBRARY_DIRS} )
endif()

set(boxes_tests
  boxes_dt.cc
  boxes_model_count.cc
)
set(GZ_BUILD_TESTS_EXTRA_EXE_SRCS boxes.cc)
gz_build_tests(${boxes_tests})

set(tests
  joints.cc
)

gz_build_tests(${tests})

if (ENABLE_TESTS_COMPILATION)
  # Increase timeout
  set_tests_properties(${TEST_TYPE}_boxes_dt PROPERTIES TIMEOUT 500)
  set_tests_properties(${TEST_TYPE}_boxes_model_count PROPERTIES TIMEOUT 2500)
  set_tests_properties(${TEST_TYPE}_joints PROPERTIES TIMEOUT 10000)
endif()

set(display_tests
)

# Build the display tests (need extra sources to compile)
gz_build_display_tests(${display_tests})

set(dri_tests
)

gz_build_dri_tests(${dri_tests})

