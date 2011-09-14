include (${gazebo_cmake_dir}/GazeboUtils.cmake)
include ( ${QT_USE_FILE} )

link_directories(  
  ${BOOST_LIBRARY_DIRS} 
)

add_definitions( ${QT_DEFINITIONS} )

add_subdirectory(qtpropertybrowser)

#include_directories ( ${CMAKE_CURRENT_BINARY_DIR})
#include_directories ( ${CMAKE_CURRENT_BINARY_DIR}/qtpropertybrowser)
#include_directories( ${QT_INCLUDE_DIR} )
#
#set(qtpropertyeditor_SRCS qtpropertybrowser/qtpropertybrowser.cpp
#                          qtpropertybrowser/qtpropertymanager.cpp
#                          qtpropertybrowser/qteditorfactory.cpp
#                          qtpropertybrowser/qtvariantproperty.cpp
#                          qtpropertybrowser/qttreepropertybrowser.cpp
#                          qtpropertybrowser/qtbuttonpropertybrowser.cpp
#                          qtpropertybrowser/qtgroupboxpropertybrowser.cpp
#                          qtpropertybrowser/qtpropertybrowserutils.cpp
#)
#
#qt4_generate_moc( qtpropertybrowser/qtpropertybrowser.h
#  ${CMAKE_CURRENT_BINARY_DIR}/qtpropertybrowser/moc_qtpropertybrowser.cpp
#)
#qt4_generate_moc( qtpropertybrowser/qtpropertymanager.h
#  ${CMAKE_CURRENT_BINARY_DIR}/qtpropertybrowser/moc_qtpropertymanager.cpp
#)
#qt4_generate_moc( qtpropertybrowser/qteditorfactory.h
#  ${CMAKE_CURRENT_BINARY_DIR}/qtpropertybrowser/moc_qteditorfactory.cpp
#)
#qt4_generate_moc( qtpropertybrowser/qtvariantproperty.h
#  ${CMAKE_CURRENT_BINARY_DIR}/qtpropertybrowser/moc_qtvariantproperty.cpp
#)
#qt4_generate_moc( qtpropertybrowser/qttreepropertybrowser.h
#  ${CMAKE_CURRENT_BINARY_DIR}/qtpropertybrowser/moc_qttreepropertybrowser.cpp
#)
#qt4_generate_moc( qtpropertybrowser/qtbuttonpropertybrowser.h
#  ${CMAKE_CURRENT_BINARY_DIR}/qtpropertybrowser/moc_qtbuttonpropertybrowser.cpp
#)
#qt4_generate_moc( qtpropertybrowser/qtgroupboxpropertybrowser.h
#  ${CMAKE_CURRENT_BINARY_DIR}/qtpropertybrowser/moc_qtgroupboxpropertybrowser.cpp
#)
#qt4_generate_moc( qtpropertybrowser/qtpropertymanager.cpp
#  ${CMAKE_CURRENT_BINARY_DIR}/qtpropertybrowser/qtpropertymanager.moc
#)
#qt4_generate_moc( qtpropertybrowser/qteditorfactory.cpp
#  ${CMAKE_CURRENT_BINARY_DIR}/qtpropertybrowser/qteditorfactory.moc
#)
#qt4_generate_moc( qtpropertybrowser/qttreepropertybrowser.cpp
#  ${CMAKE_CURRENT_BINARY_DIR}/qtpropertybrowser/qttreepropertybrowser.moc
#)
#
#set( qtpropertyeditor_HEADERS_ONLY_MOC
#  ${CMAKE_CURRENT_BINARY_DIR}/qtpropertybrowser/moc_qtpropertybrowser.cpp
#  ${CMAKE_CURRENT_BINARY_DIR}/qtpropertybrowser/moc_qtpropertymanager.cpp
#  ${CMAKE_CURRENT_BINARY_DIR}/qtpropertybrowser/moc_qteditorfactory.cpp
#  ${CMAKE_CURRENT_BINARY_DIR}/qtpropertybrowser/moc_qtvariantproperty.cpp
#  ${CMAKE_CURRENT_BINARY_DIR}/qtpropertybrowser/moc_qttreepropertybrowser.cpp
#  ${CMAKE_CURRENT_BINARY_DIR}/qtpropertybrowser/moc_qtbuttonpropertybrowser.cpp
#  ${CMAKE_CURRENT_BINARY_DIR}/qtpropertybrowser/moc_qtgroupboxpropertybrowser.cpp
#  ${CMAKE_CURRENT_BINARY_DIR}/qtpropertybrowser/qtpropertymanager.moc
#  ${CMAKE_CURRENT_BINARY_DIR}/qtpropertybrowser/qteditorfactory.moc
#  ${CMAKE_CURRENT_BINARY_DIR}/qtpropertybrowser/qttreepropertybrowser.moc
#)
#
#set_source_files_properties(
#  ${qtpropertyeditor_HEADERS_ONLY_MOC}
#  PROPERTIES
#    HEADER_FILE_ONLY true
#)
#
#qt4_add_resources(
#  qtpropertyeditor_RESOURCES
#  qtpropertybrowser/qtpropertybrowser.qrc
#)
#
#qt4_add_resources(RESOURCE_SRCS qtpropertybrowser/qtpropertybrowser.qrc)
#
#set(SRCS ${qtpropertyeditor_SRCS}
#         ${qtpropertyeditor_RESOURCES}
#)


set (sources main.cc 
             Gui.cc 
             MainWindow.cc 
             RenderWidget.cc 
             GLWidget.cc 
             TimePanel.cc
             InsertModelWidget.cc
             ModelListWidget.cc
             WorldPropertiesWidget.cc
             EntityMaker.cc
             BoxMaker.cc
             SphereMaker.cc
             CylinderMaker.cc
             GuiEvents.cc
             ModelEditWidget.cc
)

set (qt_headers MainWindow.hh 
                RenderWidget.hh 
                GLWidget.hh 
                TimePanel.hh
                InsertModelWidget.hh
                ModelListWidget.hh
                WorldPropertiesWidget.hh
                ModelEditWidget.hh
)

set (headers Gui.hh
             EntityMaker.hh
             BoxMaker.hh
             SphereMaker.hh
             CylinderMaker.hh
             GuiEvents.hh
)

set (resources images.qrc)

QT4_WRAP_CPP(headers_MOC ${qt_headers})
QT4_ADD_RESOURCES(resources_RCC ${resources})

add_executable(gzclient ${sources} ${headers_MOC} ${headers} ${resources_RCC})

target_link_libraries( gzclient gazebo_common
                                gazebo_transport
                                gazebo_sdf_interface
                                gazebo_sdf_parser
                                gazebo_rendering
                                gazebo_msgs
                                Xpm 
                                ${QT_LIBRARIES} 
                                ${ogre_libraries} 
                                ${boost_libraries} 
                                pthread)

add_definitions(${QT_DEFINITIONS})

INSTALL_EXECUTABLE( gzclient )


