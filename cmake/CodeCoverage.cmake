# Check prereqs
find_program(LCOV_PATH lcov)
find_program(GENHTML_PATH genhtml)

if(NOT CMAKE_COMPILER_IS_GNUCXX)
	# Clang version 3.0.0 and greater now supports gcov as well.
	message(WARNING "Compiler is not GNU gcc! Clang Version 3.0.0 and greater supports gcov as well, but older versions don't.")
	
	if(NOT "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
		message(FATAL_ERROR "Compiler is not GNU gcc! Aborting...")
	endif()
endif() # NOT CMAKE_COMPILER_IS_GNUCXX

if (NOT (CMAKE_BUILD_TYPE STREQUAL "Debug" OR
         CMAKE_BUILD_TYPE STREQUAL "Coverage"))
  message( WARNING "Code coverage results with an optimized (non-Debug) "
    "build may be misleading" )
endif() # NOT CMAKE_BUILD_TYPE STREQUAL "Debug"

# Param _targetname The name of new the custom make target
# Param _outputname lcov output is generated as _outputname.info
#                   HTML report is generated in _outputname/index.html
function(setup_target_for_coverage _targetname _outputname)

	if(NOT LCOV_PATH)
		message(FATAL_ERROR "lcov not found! Aborting...")
	endif() # NOT LCOV_PATH

	if(NOT GENHTML_PATH)
		message(FATAL_ERROR "genhtml not found! Aborting...")
	endif() # NOT GENHTML_PATH

	# Setup target
	add_custom_target(${_targetname}
    COMMAND ${LCOV_PATH} -q --zerocounters --directory ${PROJECT_BINARY_PATH}/gazebo
		# Capturing lcov counters and generating report
    COMMAND ${LCOV_PATH} --quiet --no-checksum --directory ${PROJECT_BINARY_DIR}/gazebo --capture --output-file ${_outputname}.info 2>/dev/null
    COMMAND ${LCOV_PATH} -q --remove ${_outputname}.info 'include/sdformat*' 'include/SimTK*' 'include/simbody*' 'include/simmath*' 'deps/*' 'build/*' 'test/*' '/usr/*' --output-file ${_outputname}.info.cleaned
		COMMAND ${GENHTML_PATH} -q --legend -o ${_outputname} ${_outputname}.info.cleaned
		COMMAND ${CMAKE_COMMAND} -E remove ${_outputname}.info
      ${_outputname}.info.cleaned
		
		WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
		COMMENT "Resetting code coverage counters to zero.\n"
      "Processing code coverage counters and generating report."
	)
	
  # Show info where to find the report. And cleanup
	add_custom_command(TARGET ${_targetname} POST_BUILD
    COMMAND ;
    COMMENT "Open ./${_outputname}/index.html in your browser to view the coverage report.")

endfunction() # SETUP_TARGET_FOR_COVERAGE
