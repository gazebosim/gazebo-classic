FIND_PACKAGE (Ruby)

if(RUBY_LIBRARY AND RUBY_INCLUDE_PATH)
    SET(RUBY_FOUND TRUE)
    message(STATUS "Found Ruby libraries: ${RUBY_LIBRARY}")
    message(STATUS "Found Ruby includes: ${RUBY_INCLUDE_PATH}")
endif (RUBY_LIBRARY AND RUBY_INCLUDE_PATH)

  IF(RUBY_FOUND)
    OPTION (BUILD_RUBY_BINDINGS "Build the Ruby bindings for libgazebo " ON)

    IF (BUILD_RUBY_BINDINGS)
        FIND_PACKAGE (SWIG)
        IF (SWIG_FOUND)
            MESSAGE (STATUS "Ruby bindings for Gazebo will be built")

            INCLUDE (${SWIG_USE_FILE})

            SET (RUBY_INTERFACE_FILE "${CMAKE_CURRENT_SOURCE_DIR}/gazeboc.i")
            SET (RUBY_MODULE_NAME gazeboc)
            SET (LIBGAZEBO_DIR ${PROJECT_SOURCE_DIR}/libgazebo)

            INCLUDE_DIRECTORIES (${RUBY_INCLUDE_PATH})
            INCLUDE_DIRECTORIES (${PROJECT_SOURCE_DIR})
            INCLUDE_DIRECTORIES (${PROJECT_BINARY_DIR})
            INCLUDE_DIRECTORIES (${LIBGAZEBO_DIR})
            INCLUDE_DIRECTORIES( ${boost_include_dirs} )

            LINK_DIRECTORIES(${boost_library_dirs} ${PROJECT_BINARY_DIR}/libgazebo)

            SET (CMAKE_SWIG_FLAGS "-w801 -c++ -I$LIBGAZEBO_DIR")

            SET_SOURCE_FILES_PROPERTIES(${RUBY_INTERFACE_FILE} PROPERTIES CPLUSPLUS ON)

            SWIG_ADD_MODULE (${RUBY_MODULE_NAME} ruby ${RUBY_INTERFACE_FILE} )
            SWIG_LINK_LIBRARIES (${RUBY_MODULE_NAME} ${RUBY_LIBRARY})
      
            ADD_DEPENDENCIES (${RUBY_MODULE_NAME} ${RUBY_INTERFACE_FILE})
            ADD_DEPENDENCIES (${RUBY_MODULE_NAME} ${LIBGAZEBO_DIR}/gazebo.h)
            TARGET_LINK_LIBRARIES (${RUBY_MODULE_NAME} gazebo)

            #TODO: find ruby in windows, check install dir is correct
            STRING(REPLACE "/usr/" "" ruby_relative_sitelib_dir ${RUBY_SITELIB_DIR})
            STRING(REPLACE "local/" "" ruby_relative_sitelib_dir ${ruby_relative_sitelib_dir})
            SET (RUBY_BINDINGS_INSTALL_DIR ${ruby_relative_sitelib_dir} CACHE PATH "Ruby bindings installation directory under $RUBY_SITELIB_DIR")
            MARK_AS_ADVANCED (RUBY_BINDINGS_INSTALL_DIR)

            INSTALL (FILES ${CMAKE_CURRENT_BINARY_DIR}/gazeboc.so
                DESTINATION ${RUBY_BINDINGS_INSTALL_DIR} )
                
        ELSE (SWIG_FOUND)
            MESSAGE (STATUS "Ruby bindings for Gazebo will not be built - could not find Swig")
        ENDIF (SWIG_FOUND)
    ELSE (BUILD_RUBY_BINDINGS)
        MESSAGE (STATUS "Ruby bindings for Gazebo will not be built - disabled by user")
    ENDIF (BUILD_RUBY_BINDINGS)
ELSE (RUBY_FOUND)
    MESSAGE (STATUS "Ruby bindings for Gazebo will not be built - could not find Ruby")
ENDIF (RUBY_FOUND)


