include (${gazebo_cmake_dir}/GazeboUtils.cmake)

  ########################################
  # Find urdfdom and urdfdom_headers
  pkg_check_modules(urdfdom_headers urdfdom_headers)
  if (NOT urdfdom_headers_FOUND)
    BUILD_WARNING ("urdfdom_headers not found, urdf parser will not be built.")
  endif ()
  if (urdfdom_headers_FOUND)
    set (HAVE_URDFDOM_HEADERS TRUE)
  endif ()

  pkg_check_modules(urdfdom urdfdom)
  if (NOT urdfdom_FOUND)
    BUILD_WARNING ("urdfdom not found, urdf parser will not be built.")
  endif ()
  if (urdfdom_FOUND)
    set (HAVE_URDFDOM TRUE)
  endif ()

  pkg_check_modules(console_bridge console_bridge)
  if (NOT console_bridge_FOUND)
    BUILD_WARNING ("console_bridge not found, urdf parser will not be built.")
  endif ()
  if (console_bridge_FOUND)
    set (HAVE_CONSOLE_BRIDGE TRUE)
  endif ()

########################################
# Find urdfdom_headers
IF (NOT HAVE_URDFDOM_HEADERS)
  SET (urdfdom_search_path /usr/include)
  FIND_PATH(URDFDOM_HEADERS_PATH urdf_model/model.h ${urdfdom_search_path})
  IF (NOT URDFDOM_HEADERS_PATH)
    MESSAGE (STATUS "Looking for urdf_model/model.h - not found")
    BUILD_WARNING ("model.h not found. urdf parser will not be built")
  ELSE (NOT URDFDOM_HEADERS_PATH)
    MESSAGE (STATUS "Looking for model.h - found")
    SET (HAVE_URDFDOM_HEADERS TRUE)
    SET (URDFDOM_HEADERS_PATH /usr/include)
  ENDIF (NOT URDFDOM_HEADERS_PATH)

ELSE (NOT HAVE_URDFDOM_HEADERS)

  SET (URDFDOM_HEADERS_PATH /usr/include)
  MESSAGE (STATUS "found urdf_model/model.h - found")

ENDIF (NOT HAVE_URDFDOM_HEADERS)

########################################
# Find urdfdom
IF (NOT HAVE_URDFDOM)
  SET (urdfdom_search_path 
    /usr/include /usr/local/include 
    /usr/include/urdf_parser
  )
  
  FIND_PATH(URDFDOM_PATH urdf_parser.h ${urdfdom_search_path})
  IF (NOT URDFDOM_PATH)
    MESSAGE (STATUS "Looking for urdf_parser/urdf_parser.h - not found")
    BUILD_WARNING ("urdf_parser.h not found. urdf parser will not be built")
    SET (URDFDOM_PATH /usr/include)
  ELSE (NOT URDFDOM_PATH)
    MESSAGE (STATUS "Looking for urdf_parser.h - found")
    SET (HAVE_URDFDOM TRUE)
    SET (URDFDOM_PATH /usr/include)
  ENDIF (NOT URDFDOM_PATH)

ELSE (NOT HAVE_URDFDOM)

  MESSAGE (STATUS "found urdf_parser/urdf_parser.h - found")

ENDIF (NOT HAVE_URDFDOM)

########################################
# Find console_bridge
IF (NOT HAVE_CONSOLE_BRIDGE)
  SET (console_bridge_search_path 
    /usr/include /usr/local/include 
  )
  
  FIND_PATH(CONSOLE_BRIDGE_PATH console_bridge/console.h ${console_bridge_search_path})
  IF (NOT CONSOLE_BRIDGE_PATH)
    MESSAGE (STATUS "Looking for console_bridge/console.h - not found")
    BUILD_WARNING ("console.h not found. urdf parser (depends on console_bridge) will not be built")
    SET (CONSOLE_BRIDGE_PATH /usr/include)
  ELSE (NOT CONSOLE_BRIDGE_PATH)
    MESSAGE (STATUS "Looking for console.h - found")
    SET (HAVE_CONSOLE_BRIDGE TRUE)
    SET (CONSOLE_BRIDGE_PATH /usr/include)
  ENDIF (NOT CONSOLE_BRIDGE_PATH)

ELSE (NOT HAVE_CONSOLE_BRIDGE)

  MESSAGE (STATUS "found console_bridge/console.h - found")

ENDIF (NOT HAVE_CONSOLE_BRIDGE)



set (sources
  Console.cc
  Converter.cc
  parser.cc
  Param.cc
  SDF.cc
)

set (headers
  Console.hh
  Converter.hh
  Param.hh
  parser.hh
  SDF.hh
)

if (HAVE_URDFDOM AND HAVE_URDFDOM_HEADERS)
  include_directories(SYSTEM ${urdfdom_INCLUDE_DIRS}
                             ${urdfdom_headers_INCLUDE_DIRS}
                             ${console_bridge_INCLUDE_DIRS}
                             ${CMAKE_SOURCE_DIR}/deps/opende/include)

  link_directories(${urdfdom_LIBRARY_DIRS}
                   ${urdfdom_headers_LIBRARY_DIRS}
                   ${console_bridge_LIBRARY_DIRS})

  set (sources ${sources} parser_urdf.cc)
  set (headers ${headers} parser_urdf.hh)

  # add_definitions(${urdfdom_LDFLAGS}
  # ${urdfdom_headers_LDFLAGS} ${console_bridge_LDFLAGS})
endif(HAVE_URDFDOM AND HAVE_URDFDOM_HEADERS)

gz_add_library(gazebo_sdf_interface ${sources})

if (HAVE_URDFDOM AND HAVE_URDFDOM_HEADERS)
  target_link_libraries(gazebo_sdf_interface
    gazebo_common
    ${tinyxml_libraries}
    ${Boost_LIBRARIES}
    gazebo_ode
    ${urdfdom_headers_LIBRARIES}
    ${urdfdom_LIBRARIES}
    ${console_bridge_LIBRARIES}
    ${urdfdom_LDFLAGS}
    ${console_bridge_LDFLAGS})
else(HAVE_URDFDOM AND HAVE_URDFDOM_HEADERS)
  target_link_libraries(gazebo_sdf_interface
    gazebo_common ${tinyxml_libraries} ${Boost_LIBRARIES})
endif(HAVE_URDFDOM AND HAVE_URDFDOM_HEADERS)

gz_install_library(gazebo_sdf_interface)
gz_install_includes("sdf/interface" ${headers})
