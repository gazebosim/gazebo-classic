add_definitions(-DWITH_CGRAPH)
add_definitions(-DHAVE_STRING_H)

# Activate Export macro for building the library itself
# if QGVCORE_LIB is not set or equal 0 (null), import macro is used
add_definitions(-DQGVCORE_LIB)

include ( ${QT_USE_FILE} )

SET(sources
  private/QGVCore.cpp
  private/QGVGraphPrivate.cpp
  private/QGVEdgePrivate.cpp
  private/QGVGvcPrivate.cpp
  private/QGVNodePrivate.cpp
  QGVEdge.cpp
  QGVNode.cpp
  QGVScene.cpp
  QGVSubGraph.cpp
)

SET(qt_headers
  QGVScene.h
)

SET(headers
  qgv.h
  QGVEdge.h
  QGVNode.h
  QGVSubGraph.h
)

add_definitions(${QT_DEFINITIONS})

QT4_WRAP_CPP(headers_MOC ${qt_headers})

include_directories(${GRAPHVIZ_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/private)

add_library(gazebo_gui_model_qgv STATIC ${sources} ${headers_MOC} ${headers})

set_target_properties(gazebo_gui_model_qgv PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_link_libraries(gazebo_gui_model_qgv
  ${QT_LIBRARIES}
  ${GRAPHVIZ_LIBRARIES}
)
