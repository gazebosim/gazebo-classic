
################################################################################
#APPEND_TO_CACHED_STRING(_string _cacheDesc [items...])
# Appends items to a cached list.
MACRO (APPEND_TO_CACHED_STRING _string _cacheDesc)
  FOREACH (newItem ${ARGN})
    SET (${_string} "${${_string}} ${newItem}" CACHE INTERNAL ${_cacheDesc} FORCE)
  ENDFOREACH (newItem ${ARGN})
  #STRING(STRIP ${${_string}} ${_string})
ENDMACRO (APPEND_TO_CACHED_STRING)
                 
################################################################################
# APPEND_TO_CACHED_LIST (_list _cacheDesc [items...]
# Appends items to a cached list.
MACRO (APPEND_TO_CACHED_LIST _list _cacheDesc)
  SET (tempList ${${_list}})
  FOREACH (newItem ${ARGN})
    LIST (APPEND tempList ${newItem})
  ENDFOREACH (newItem ${newItem})
  SET (${_list} ${tempList} CACHE INTERNAL ${_cacheDesc} FORCE)
ENDMACRO(APPEND_TO_CACHED_LIST)

###############################################################################
# Append sources to the server sources list
MACRO (APPEND_TO_SERVER_SOURCES)
  FOREACH (src ${ARGN})
    APPEND_TO_CACHED_LIST(gazeboserver_sources 
                          ${gazeboserver_sources_desc}                   
                          ${CMAKE_CURRENT_SOURCE_DIR}/${src})
  ENDFOREACH (src ${ARGN})
ENDMACRO (APPEND_TO_SERVER_SOURCES)

###############################################################################
# Append headers to the server headers list
MACRO (APPEND_TO_SERVER_HEADERS)
  FOREACH (src ${ARGN})
    APPEND_TO_CACHED_LIST(gazeboserver_headers
                          ${gazeboserver_headers_desc}                   
                          ${CMAKE_CURRENT_SOURCE_DIR}/${src})
    APPEND_TO_CACHED_LIST(gazeboserver_headers_nopath
                          "gazeboserver_headers_nopath"                   
                          ${src})
  ENDFOREACH (src ${ARGN})
ENDMACRO (APPEND_TO_SERVER_HEADERS)

###############################################################################
# Append sources to the sensor sources list
MACRO (APPEND_TO_SENSOR_SOURCES)
  FOREACH (src ${ARGN})
    APPEND_TO_CACHED_LIST(gazebosensor_sources 
                          ${gazebosensor_sources_desc}                   
                          ${CMAKE_CURRENT_SOURCE_DIR}/${src})
  ENDFOREACH (src ${ARGN})
ENDMACRO (APPEND_TO_SENSOR_SOURCES)

###############################################################################
# Append sources to the controller sources list
MACRO (APPEND_TO_CONTROLLER_SOURCES)
  FOREACH (src ${ARGN})
    APPEND_TO_CACHED_LIST(gazebocontroller_sources 
                          ${gazebocontroller_sources_desc}                   
                          ${CMAKE_CURRENT_SOURCE_DIR}/${src})
  ENDFOREACH (src ${ARGN})
ENDMACRO (APPEND_TO_CONTROLLER_SOURCES)


#################################################
# Macro to turn a list into a string (why doesn't CMake have this built-in?)
MACRO (LIST_TO_STRING _string _list)
    SET (${_string})
    FOREACH (_item ${_list})
      SET (${_string} "${${_string}} ${_item}")
    ENDFOREACH (_item)
    #STRING(STRIP ${${_string}} ${_string})
ENDMACRO (LIST_TO_STRING)

#################################################
# BUILD ERROR macro
macro (BUILD_ERROR)
  foreach (str ${ARGN})
    SET (msg "\t${str}")
    MESSAGE (STATUS ${msg})
    APPEND_TO_CACHED_LIST(build_errors "build errors" ${msg})
  endforeach ()
endmacro (BUILD_ERROR)

#################################################
# BUILD WARNING macro
macro (BUILD_WARNING)
  foreach (str ${ARGN})
    SET (msg "\t${str}" )
    MESSAGE (STATUS ${msg} )
    APPEND_TO_CACHED_LIST(build_warnings "build warning" ${msg})
  endforeach (str ${ARGN})
endmacro (BUILD_WARNING)

#################################################
macro (gz_add_library _name)
  add_library(${_name} SHARED ${ARGN})
  target_link_libraries (${_name} ${general_libraries})
endmacro ()

#################################################
macro (gz_add_executable _name)
  add_executable(${_name} ${ARGN})
  target_link_libraries (${_name} ${general_libraries})
endmacro ()


#################################################
macro (gz_install_includes _subdir)
  install(FILES ${ARGN} DESTINATION ${INCLUDE_INSTALL_DIR}/${_subdir} COMPONENT headers)
endmacro()

#################################################
macro (gz_install_library _name)
  set_target_properties(${_name} PROPERTIES SOVERSION ${GAZEBO_MAJOR_VERSION} VERSION ${GAZEBO_VERSION_FULL})
  install (TARGETS ${_name} DESTINATION ${LIB_INSTALL_DIR} COMPONENT shlib)
endmacro ()

#################################################
macro (gz_install_executable _name)
  set_target_properties(${_name} PROPERTIES VERSION ${GAZEBO_VERSION_FULL})
  install (TARGETS ${_name} DESTINATION ${BIN_INSTALL_DIR})
endmacro ()

#################################################
macro (gz_setup_unix)
endmacro()

#################################################
macro (gz_setup_windows)
endmacro()

#################################################
macro (gz_setup_apple)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-undefined -Wl,dynamic_lookup")
endmacro()

# This should be migrated to more fine control solution based on set_property APPEND
# directories. It's present on cmake 2.8.8 while precise version is 2.8.7  
link_directories(${PROJECT_BINARY_DIR}/test)
include_directories("${PROJECT_SOURCE_DIR}/test/gtest/include")

#################################################
# Hack: extra sources to build binaries can be supplied to gz_build_tests in the variable
#       GZ_BUILD_TESTS_EXTRA_EXE_SRCS. This variable will be clean up at the end of the function
macro (gz_build_tests)
  # Build all the tests
  foreach(GTEST_SOURCE_file ${ARGN})
    string(REGEX REPLACE ".cc" "" BINARY_NAME ${GTEST_SOURCE_file})
    if(USE_LOW_MEMORY_TESTS)
      add_definitions(-DUSE_LOW_MEMORY_TESTS=1)
    endif(USE_LOW_MEMORY_TESTS)
    add_executable(${BINARY_NAME} ${GTEST_SOURCE_file} ${GZ_BUILD_TESTS_EXTRA_EXE_SRCS})

    add_dependencies(${BINARY_NAME}
      gtest gtest_main
      gazebo_common
      gazebo_math
      gazebo_physics
      gazebo_sensors
      gazebo_rendering
      gazebo_msgs
      gazebo_transport
      )

    target_link_libraries(${BINARY_NAME}
      libgtest.a
      libgtest_main.a
      gazebo_common
      gazebo_math
      gazebo_physics
      gazebo_sensors
      gazebo_rendering
      gazebo_msgs
      gazebo_transport
      libgazebo
      pthread
      )

    add_test(${BINARY_NAME} ${CMAKE_CURRENT_BINARY_DIR}/${BINARY_NAME}
	--gtest_output=xml:${CMAKE_BINARY_DIR}/test_results/${TEST_TYPE}_${BINARY_NAME}.xml)
  
    set_tests_properties(${BINARY_NAME} PROPERTIES TIMEOUT 240)
  
    # Check that the test produced a result and create a failure if it didn't.
    # Guards against crashed and timed out tests.
    add_test(check_${BINARY_NAME} ${PROJECT_SOURCE_DIR}/tools/check_test_ran.py
	${CMAKE_BINARY_DIR}/test_results/${TEST_TYPE}_${BINARY_NAME}.xml)
  endforeach()

  set(GZ_BUILD_TESTS_EXTRA_EXE_SRCS "")
endmacro()

#################################################

# Define GUI testing macros as empty and redefine them if support is found
macro (gz_build_qt_tests)
endmacro()
macro (gz_build_display_tests)
endmacro()
macro (gz_build_dri_tests)
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
      gazebo_gui
      gazebo_common
      gazebo_math
      gazebo_physics
      gazebo_sensors
      gazebo_rendering
      gazebo_msgs
      gazebo_transport
      libgazebo
      pthread
      ${QT_QTTEST_LIBRARY}
      ${QT_LIBRARIES}
      )

    # QTest need and extra -o parameter to write logging information to a file
    add_test(${BINARY_NAME} ${CMAKE_CURRENT_BINARY_DIR}/${BINARY_NAME}
	-xml -o ${CMAKE_BINARY_DIR}/test_results/${TEST_TYPE}_${BINARY_NAME}.xml)

    set_tests_properties(${BINARY_NAME} PROPERTIES TIMEOUT 240)

    # Check that the test produced a result and create a failure if it didn't.
    # Guards against crashed and timed out tests.
    add_test(check_${BINARY_NAME} ${PROJECT_SOURCE_DIR}/tools/check_test_ran.py
	${CMAKE_BINARY_DIR}/test_results/${TEST_TYPE}_${BINARY_NAME}.xml)
    endforeach()
  endmacro()
endif()

if (VALID_DRI_DISPLAY)
  macro (gz_build_dri_tests)
    gz_build_tests(${ARGV})
  endmacro()
endif()
