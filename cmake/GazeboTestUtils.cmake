#################################################
# Hack: extra sources to build binaries can be supplied to gz_build_tests in
# the variable GZ_BUILD_TESTS_EXTRA_EXE_SRCS. This variable will be clean up
# at the end of the function
macro (gz_build_tests)
  # Build all the tests
  foreach(GTEST_SOURCE_file ${ARGN})
    string(REGEX REPLACE "\\.cc" "" BINARY_NAME ${GTEST_SOURCE_file})
    set(BINARY_NAME ${TEST_TYPE}_${BINARY_NAME})
    if(USE_LOW_MEMORY_TESTS)
      add_definitions(-DUSE_LOW_MEMORY_TESTS=1)
    endif(USE_LOW_MEMORY_TESTS)
    add_executable(${BINARY_NAME} ${GTEST_SOURCE_file}
                   ${GZ_BUILD_TESTS_EXTRA_EXE_SRCS})


    link_directories(${PROJECT_BINARY_DIR}/test)
    add_dependencies(${BINARY_NAME}
      gtest
      gtest_main
      gazebo_common
      gazebo_math
      gazebo_physics
      gazebo_sensors
      gazebo_rendering
      gazebo_msgs
      gazebo_transport
      gazebo_test_fixture
      )


    target_link_libraries(${BINARY_NAME}
      # This two libraries are need to workaround on bug 
      # https://bitbucket.org/osrf/gazebo/issue/1516
      gazebo_physics
      gazebo_sensors
      # libgazebo will bring all most of gazebo libraries as dependencies
      libgazebo
      gazebo_test_fixture
      gtest
      gtest_main
      )

    add_test(${BINARY_NAME} ${CMAKE_CURRENT_BINARY_DIR}/${BINARY_NAME}
	--gtest_output=xml:${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml)

    set(_env_vars)
    list(APPEND _env_vars "CMAKE_PREFIX_PATH=${CMAKE_BINARY_DIR}:${CMAKE_PREFIX_PATH}")
    list(APPEND _env_vars "GAZEBO_PLUGIN_PATH=${CMAKE_BINARY_DIR}/plugins:${CMAKE_BINARY_DIR}/plugins/events:${CMAKE_BINARY_DIR}/plugins/rest_web")
    list(APPEND _env_vars "GAZEBO_RESOURCE_PATH=${CMAKE_SOURCE_DIR}")
    list(APPEND _env_vars "PATH=${CMAKE_BINARY_DIR}/gazebo:${CMAKE_BINARY_DIR}/tools:$ENV{PATH}")
    list(APPEND _env_vars "PKG_CONFIG_PATH=${CMAKE_BINARY_DIR}/cmake/pkgconfig:$PKG_CONFIG_PATH")
    set_tests_properties(${BINARY_NAME} PROPERTIES
      TIMEOUT 240
      ENVIRONMENT "${_env_vars}")

    # Check that the test produced a result and create a failure if it didn't.
    # Guards against crashed and timed out tests.
    add_test(check_${BINARY_NAME} ${PROJECT_SOURCE_DIR}/tools/check_test_ran.py
	${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml)
  endforeach()

  set(GZ_BUILD_TESTS_EXTRA_EXE_SRCS "")
endmacro()

if (VALID_DISPLAY)
  # Redefine build display tests
  macro (gz_build_display_tests)
    gz_build_tests(${ARGV})
  endmacro()

  # Redefine build qt tests
  macro (gz_build_qt_tests)
   # Build all the tests
   foreach(QTEST_SOURCE_file ${ARGN})
     string(REGEX REPLACE ".cc" "" BINARY_NAME ${QTEST_SOURCE_file})
     string(REGEX REPLACE ".cc" ".hh" QTEST_HEADER_file ${QTEST_SOURCE_file})
     set(BINARY_NAME ${TEST_TYPE}_${BINARY_NAME})
     QT4_WRAP_CPP(${BINARY_NAME}_MOC ${QTEST_HEADER_file} ${CMAKE_SOURCE_DIR}/gazebo/gui/QTestFixture.hh)

     add_executable(${BINARY_NAME}
      ${${BINARY_NAME}_MOC} ${QTEST_SOURCE_file} ${CMAKE_SOURCE_DIR}/gazebo/gui/QTestFixture.cc)

    add_dependencies(${BINARY_NAME}
      gazebo_gui
      gazebo_common
      gazebo_math
      gazebo_physics
      gazebo_sensors
      gazebo_rendering
      gazebo_msgs
      gazebo_transport
      )

    target_link_libraries(${BINARY_NAME}
      # This two libraries are need to workaround on bug 
      # https://bitbucket.org/osrf/gazebo/issue/1516
      gazebo_physics
      gazebo_sensors
      # gazebo_gui and libgazebo will bring all most of gazebo libraries as dependencies
      libgazebo
      gazebo_gui
      ${QT_QTTEST_LIBRARY}
      ${QT_LIBRARIES}
      )

    # QTest need and extra -o parameter to write logging information to a file
    add_test(${BINARY_NAME} ${CMAKE_CURRENT_BINARY_DIR}/${BINARY_NAME}
	-xml -o ${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml)

    set_tests_properties(${BINARY_NAME} PROPERTIES TIMEOUT 240)

    # Check that the test produced a result and create a failure if it didn't.
    # Guards against crashed and timed out tests.
    add_test(check_${BINARY_NAME} ${PROJECT_SOURCE_DIR}/tools/check_test_ran.py
	${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml)
    endforeach()
  endmacro()
endif()

if (VALID_DRI_DISPLAY)
  macro (gz_build_dri_tests)
    gz_build_tests(${ARGV})
  endmacro()
endif()
