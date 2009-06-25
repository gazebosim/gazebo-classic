INCLUDE (FindPkgConfig)

SET(REQUIRED_ODE_VERSION ${ODE_VERSION})

########################################
# Find packages
IF (PKG_CONFIG_FOUND)
  pkg_check_modules(ODE ode>=${ODE_VERSION})
ENDIF (PKG_CONFIG_FOUND)

# if we didnt find it in pkg-config try ode-config
IF (NOT ODE_FOUND)
  SET(ODE_CONFIG_PATH "ode-config")
  EXEC_PROGRAM(${ODE_CONFIG_PATH} ARGS --version RETURN_VALUE ODE_CONFIG_RETURN OUTPUT_VARIABLE DETECTED_ODE_VERSION )
  STRING(REGEX REPLACE "[\r\n]" " " ${DETECTED_ODE_VERSION} "${${DETECTED_ODE_VERSION}}")

  # ode-config exists then get the details
  IF(NOT ODE_CONFIG_RETURN)
    IF(NOT ${REQUIRED_ODE_VERSION} VERSION_LESS ${DETECTED_ODE_VERSION} )
      MESSAGE(STATUS "  ode-config reports version ${DETECTED_ODE_VERSION}")
      EXEC_PROGRAM(${ODE_CONFIG_PATH} ARGS --cflags RETURN_VALUE ODE_CONFIG_RETURN OUTPUT_VARIABLE ODE_INCLUDE_DIRS )
      STRING(REGEX REPLACE "[\r\n]" " " ${ODE_INCLUDE_DIRS} "${${ODE_INCLUDE_DIRS}}")

      EXEC_PROGRAM(${ODE_CONFIG_PATH} ARGS --libs RETURN_VALUE ODE_CONFIG_RETURN OUTPUT_VARIABLE ODE_LDFLAGS )
      STRING(REGEX REPLACE "[\r\n]" " " ${ODE_LDFLAGS} "${${ODE_LDFLAGS}}")
        
      SET(ODE_FOUND ${DETECTED_ODE_VERSION})
    ELSE(NOT ${REQUIRED_ODE_VERSION} VERSION_LESS ${DETECTED_ODE_VERSION} )
      MESSAGE(STATUS "ode-config reports wrong version (${DETECTED_ODE_VERSION} < ${REQUIRED_ODE_VERSION})")
    ENDIF(NOT ${REQUIRED_ODE_VERSION} VERSION_LESS ${DETECTED_ODE_VERSION} )
  ELSE(NOT ODE_CONFIG_RETURN)
      MESSAGE(STATUS "no ode-config found")
  ENDIF(NOT ODE_CONFIG_RETURN)
ENDIF (NOT ODE_FOUND)

IF (NOT ODE_FOUND)
  MESSAGE (SEND_ERROR "\nError: ODE and development files not found. See the following website: http://www.ode.org")
ELSE (NOT ODE_FOUND)
  APPEND_TO_CACHED_LIST(gazeboserver_include_dirs 
                        ${gazeboserver_include_dirs_desc} 
                        ${ODE_INCLUDE_DIRS})
  APPEND_TO_CACHED_LIST(gazeboserver_link_dirs 
                        ${gazeboserver_link_dirs_desc} 
                        ${ODE_LIBRARY_DIRS})
  APPEND_TO_CACHED_LIST(gazeboserver_link_libs 
                        ${gazeboserver_link_libs_desc} 
                        ${ODE_LINK_LIBS})
  APPEND_TO_CACHED_LIST(gazeboserver_link_libs 
                        ${gazeboserver_link_libs_desc} 
                        ${ODE_LIBRARIES})
  APPEND_TO_CACHED_LIST(gazeboserver_link_libs 
                        ${gazeboserver_link_libs_desc} 
                        ${ODE_LDFLAGS})
ENDIF (NOT ODE_FOUND)
