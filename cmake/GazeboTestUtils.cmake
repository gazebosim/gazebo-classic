#################################################
# VAR: GZ_BUILD_TESTS_EXTRA_EXE_SRCS
# Hack: extra sources to build binaries can be supplied to gz_build_tests in
# the variable GZ_BUILD_TESTS_EXTRA_EXE_SRCS. This variable will be clean up
# at the end of the function
#
# ARG: EXTRA_LIBS
# List extra libraries that the sources should be linked against after the
# EXTRA_LIBS tag. Example:
# gz_build_tests(${test_sources} EXTRA_LIBS ${test_libraries})
#
macro (gz_build_tests)
  set(_append_sources TRUE)

  set(_sources)
  set(_extra_libs)

  foreach(arg ${ARGN})
    if ("${arg}" STREQUAL "EXTRA_LIBS")
      set(_append_sources FALSE)
    else()
      if (_append_sources)
        list(APPEND _sources ${arg})
      else()
        list(APPEND _extra_libs ${arg})
      endif()
    endif()
  endforeach()

  # Build all the tests
  foreach(GTEST_SOURCE_file ${_sources})
    string(REGEX REPLACE "\\.cc" "" BINARY_NAME ${GTEST_SOURCE_file})
    set(BINARY_NAME ${TEST_TYPE}_${BINARY_NAME})
    if(USE_LOW_MEMORY_TESTS)
      add_definitions(-DUSE_LOW_MEMORY_TESTS=1)
    endif(USE_LOW_MEMORY_TESTS)
    if (BUILD_TESTING)
      add_executable(${BINARY_NAME}
                     ${GTEST_SOURCE_file}
                     ${GZ_BUILD_TESTS_EXTRA_EXE_SRCS})
    else()
      add_executable(${BINARY_NAME} EXCLUDE_FROM_ALL
                     ${GTEST_SOURCE_file}
                     ${GZ_BUILD_TESTS_EXTRA_EXE_SRCS})
    endif()

    link_directories(${PROJECT_BINARY_DIR}/test)
    target_link_libraries(${BINARY_NAME}
      gtest
      gtest_main
      ${_extra_libs}
    )
    if (UNIX)
      # gtest uses pthread on UNIX
      target_link_libraries(${BINARY_NAME} pthread)
    endif()
    # Visual Studio enables c++11 support by default
    if (NOT MSVC)
      if(CMAKE_VERSION VERSION_LESS 3.8.2)
        target_compile_options(${BINARY_NAME} PRIVATE -std=c++11)
      else()
        target_compile_features(${BINARY_NAME} PRIVATE cxx_std_11)
      endif()
    endif()

    add_test(${BINARY_NAME} ${CMAKE_CURRENT_BINARY_DIR}/${BINARY_NAME}
	--gtest_output=xml:${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml)

    set(_env_vars)
    list(APPEND _env_vars "GAZEBO_PLUGIN_PATH=${CMAKE_BINARY_DIR}/plugins:${CMAKE_BINARY_DIR}/plugins/events:${CMAKE_BINARY_DIR}/plugins/rest_web")
    list(APPEND _env_vars "GAZEBO_RESOURCE_PATH=${CMAKE_SOURCE_DIR}")
    list(APPEND _env_vars "PATH=${CMAKE_BINARY_DIR}/gazebo:${CMAKE_BINARY_DIR}/tools:$ENV{PATH}")
    set_tests_properties(${BINARY_NAME} PROPERTIES
      TIMEOUT 240
      ENVIRONMENT "${_env_vars}")

    # Check that the test produced a result and create a failure if it didn't.
    # Guards against crashed and timed out tests.
    add_test(check_${BINARY_NAME} ${PYTHON_EXECUTABLE} ${PROJECT_SOURCE_DIR}/tools/check_test_ran.py
      ${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml)

    if(GAZEBO_RUN_VALGRIND_TESTS AND VALGRIND_PROGRAM)
      add_test(memcheck_${BINARY_NAME} ${VALGRIND_PROGRAM} --leak-check=full
        --error-exitcode=1 --show-leak-kinds=all ${CMAKE_CURRENT_BINARY_DIR}/${BINARY_NAME})
    endif()

    add_dependencies(tests ${BINARY_NAME})
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

      if (BUILD_TESTING)
        add_executable(${BINARY_NAME}
         ${${BINARY_NAME}_MOC} ${QTEST_SOURCE_file} ${CMAKE_SOURCE_DIR}/gazebo/gui/QTestFixture.cc)
      else()
        add_executable(${BINARY_NAME} EXCLUDE_FROM_ALL
         ${${BINARY_NAME}_MOC} ${QTEST_SOURCE_file} ${CMAKE_SOURCE_DIR}/gazebo/gui/QTestFixture.cc)
      endif()

    add_dependencies(${BINARY_NAME}
      gazebo_gui
      gazebo_common
      gazebo_physics
      gazebo_sensors
      gazebo_rendering
      gazebo_msgs
      gazebo_transport
      )

    target_link_libraries(${BINARY_NAME}
      # gazebo_gui and libgazebo will bring all most of gazebo
      # libraries as dependencies
      libgazebo
      gazebo_gui
      ${Qt5Test_LIBRARIES}
      )

    # QTest need and extra -o parameter to write logging information to a file
    add_test(${BINARY_NAME} ${CMAKE_CURRENT_BINARY_DIR}/${BINARY_NAME}
      -xml -o ${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml)

    set(_env_vars)
    list(APPEND _env_vars "GAZEBO_PLUGIN_PATH=${CMAKE_BINARY_DIR}/plugins:${CMAKE_BINARY_DIR}/plugins/events:${CMAKE_BINARY_DIR}/plugins/rest_web")
    list(APPEND _env_vars "GAZEBO_RESOURCE_PATH=${CMAKE_SOURCE_DIR}")
    list(APPEND _env_vars "PATH=${CMAKE_BINARY_DIR}/gazebo:${CMAKE_BINARY_DIR}/tools:$ENV{PATH}")
    set_tests_properties(${BINARY_NAME} PROPERTIES
      TIMEOUT 240
      ENVIRONMENT "${_env_vars}")

    # Check that the test produced a result and create a failure if it didn't.
    # Guards against crashed and timed out tests.
    add_test(check_${BINARY_NAME} ${PROJECT_SOURCE_DIR}/tools/check_test_ran.py
      ${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml)

    if(GAZEBO_RUN_VALGRIND_TESTS AND VALGRIND_PROGRAM)
      add_test(memcheck_${BINARY_NAME} ${VALGRIND_PROGRAM} --leak-check=full
        --error-exitcode=1 --show-leak-kinds=all ${CMAKE_CURRENT_BINARY_DIR}/${BINARY_NAME})
    endif()

    add_dependencies(tests ${BINARY_NAME})
    endforeach()
  endmacro()
else()
  # Fake macros when no valid display is found
  macro (gz_build_display_tests)
  endmacro()
  macro (gz_build_qt_tests)
  endmacro()
endif()

if (VALID_DRI_DISPLAY)
  macro (gz_build_dri_tests)
    gz_build_tests(${ARGV})
  endmacro()
else()
  # Fake macro when no valid DRI display is found
  macro (gz_build_dri_tests)
  endmacro()
endif()
