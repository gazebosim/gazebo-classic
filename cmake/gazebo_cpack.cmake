################################################################################
#Find available package generators

# DEB
if ("${CMAKE_SYSTEM}" MATCHES "Linux")
  find_program(DPKG_PROGRAM dpkg)
  if (EXISTS ${DPKG_PROGRAM})
    list (APPEND CPACK_GENERATOR "DEB")
  endif(EXISTS ${DPKG_PROGRAM})
endif()

list (APPEND CPACK_SOURCE_GENERATOR "TBZ2")
list (APPEND CPACK_SOURCE_GENERATOR "ZIP")
list (APPEND CPACK_SOURCE_IGNORE_FILES ";Ogre.log;TODO;/.hg/;.swp$;/build/")

include (InstallRequiredSystemLibraries)

#execute_process(COMMAND dpkg --print-architecture _NPROCE)
set (DEBIAN_PACKAGE_DEPENDS "libogre-dev, libfreeimage-dev, libqt4-dev, libprotobuf-dev, libprotoc-dev, libtbb2, libxml2, libboost-all-dev")

set (GAZEBO_CPACK_CFG_FILE "${PROJECT_BINARY_DIR}/cpack_options.cmake")

################################################################################
# Make the CPack input file
#macro(GAZEBO_MAKE_CPACK_INPUT)
#  set(_cpack_cfg_in "${gazebo_cmake_dir}/cpack_options.cmake.in")
#
#  #Prepare the components list
#  #GAZEBO_CPACK_MAKE_COMPS_OPTS(GAZEBO_CPACK_COMPONENTS "${_comps}")
#
#  configure_file(${_cpack_cfg_in} ${GAZEBO_CPACK_CFG_FILE} @ONLY)
#endmacro(GAZEBO_MAKE_CPACK_INPUT)


#macro(GAZEBO_CPACK_MAKE_COMPS_OPTS _var _current)
#    set(_comps_list)
#    foreach(_ss ${GAZEBO_SUBSYSTEMS})
#        GAZEBO_GET_SUBSYS_STATUS(_status ${_ss})
#        if(_status)
#            set(_comps_list "${_comps_list} ${_ss}")
#            GAZEBO_CPACK_ADD_COMP_INFO(${_var} ${_ss})
#        endif(_status)
#    endforeach(_ss)
#    set(${_var} "${${_var}}\nset(CPACK_COMPONENTS_ALL${_comps_list})\n")
#endmacro(GAZEBO_CPACK_MAKE_COMPS_OPTS)

