include (${gazebo_cmake_dir}/GazeboUtils.cmake)
include ( ${QT_USE_FILE} )

set (sources qtpropertybrowser.cpp 
             qtpropertymanager.cpp
             qteditorfactory.cpp
             qtvariantproperty.cpp
             qttreepropertybrowser.cpp
             qtbuttonpropertybrowser.cpp
             qtgroupboxpropertybrowser.cpp
             qtpropertybrowserutils.cpp
 ) 

set (qt_headers qtpropertybrowser.h 
                qtpropertymanager.h
                qteditorfactory.h
                qtvariantproperty.h
                qttreepropertybrowser.h
                qtbuttonpropertybrowser.h
                qtgroupboxpropertybrowser.h
                qtpropertybrowserutils_p.h
  )

set (resources qtpropertybrowser.qrc)
QT4_WRAP_CPP(headers_MOC ${qt_headers})
QT4_ADD_RESOURCES(resources_RCC ${resources})

add_library(gzqtpropertybrowser STATIC ${sources} ${headers_MOC} ${resources_RCC})

#set(CMAKE_PACKAGE_QTGUI TRUE)
#
#add_definitions( ${QT_DEFINITIONS} )
#
#set(qtpropertyeditor_SRCS
#                           qtpropertybrowser.cpp
#                           qtpropertymanager.cpp
#                           qteditorfactory.cpp
#                           qtvariantproperty.cpp
#                           qttreepropertybrowser.cpp
#                           qtbuttonpropertybrowser.cpp
#                           qtgroupboxpropertybrowser.cpp
#                           qtpropertybrowserutils.cpp
#)
#
include_directories(${CMAKE_CURRENT_BINARY_DIR})
#include_directories( ${QT_INCLUDE_DIR} )
#
#qt4_generate_moc(
#  qtpropertybrowser.h
#  ${CMAKE_CURRENT_BINARY_DIR}/moc_qtpropertybrowser.cpp
#)
#qt4_generate_moc(
#  qtpropertymanager.h
#  ${CMAKE_CURRENT_BINARY_DIR}/moc_qtpropertymanager.cpp
#)
#qt4_generate_moc(
#  qteditorfactory.h
#  ${CMAKE_CURRENT_BINARY_DIR}/moc_qteditorfactory.cpp
#)
#qt4_generate_moc(
#  qtvariantproperty.h
#  ${CMAKE_CURRENT_BINARY_DIR}/moc_qtvariantproperty.cpp
#)
#qt4_generate_moc(
#  qttreepropertybrowser.h
#  ${CMAKE_CURRENT_BINARY_DIR}/moc_qttreepropertybrowser.cpp
#)
#qt4_generate_moc(
#  qtbuttonpropertybrowser.h
#  ${CMAKE_CURRENT_BINARY_DIR}/moc_qtbuttonpropertybrowser.cpp
#)
#qt4_generate_moc(
#  qtgroupboxpropertybrowser.h
#  ${CMAKE_CURRENT_BINARY_DIR}/moc_qtgroupboxpropertybrowser.cpp
#)
#qt4_generate_moc(
#  qtpropertymanager.cpp
#  ${CMAKE_CURRENT_BINARY_DIR}/qtpropertymanager.moc
#)
#qt4_generate_moc(
#  qteditorfactory.cpp
#  ${CMAKE_CURRENT_BINARY_DIR}/qteditorfactory.moc
#)
#qt4_generate_moc(
#  qttreepropertybrowser.cpp
#  ${CMAKE_CURRENT_BINARY_DIR}/qttreepropertybrowser.moc
#)
#
#set(
#  qtpropertyeditor_HEADERS_ONLY_MOC
#  ${CMAKE_CURRENT_BINARY_DIR}/moc_qtpropertybrowser.cpp
#  ${CMAKE_CURRENT_BINARY_DIR}/moc_qtpropertymanager.cpp
#  ${CMAKE_CURRENT_BINARY_DIR}/moc_qteditorfactory.cpp
#  ${CMAKE_CURRENT_BINARY_DIR}/moc_qtvariantproperty.cpp
#  ${CMAKE_CURRENT_BINARY_DIR}/moc_qttreepropertybrowser.cpp
#  ${CMAKE_CURRENT_BINARY_DIR}/moc_qtbuttonpropertybrowser.cpp
#  ${CMAKE_CURRENT_BINARY_DIR}/moc_qtgroupboxpropertybrowser.cpp
#  ${CMAKE_CURRENT_BINARY_DIR}/qtpropertymanager.moc
#  ${CMAKE_CURRENT_BINARY_DIR}/qteditorfactory.moc
#  ${CMAKE_CURRENT_BINARY_DIR}/qttreepropertybrowser.moc
#)
#
#set_source_files_properties(
#  ${qtpropertyeditor_HEADERS_ONLY_MOC}
#  PROPERTIES
#    HEADER_FILE_ONLY true
#)
#
##qt4_generate_moc(
##  qtpropertybrowserutils_p.h
##  ${CMAKE_CURRENT_BINARY_DIR}/moc_qtpropertybrowserutils_p.cpp
##)
##
##set(
##  qtpropertyeditor_HEADERS_MOC
##  ${CMAKE_CURRENT_BINARY_DIR}/moc_qtpropertybrowseutils_p.cpp
##)
##
##set(
##  qtpropertyeditor_MOC
##  ${qtpropertyeditor_HEADERS_MOC}
##  ${qtpropertyeditor_HEADERS_ONLY_MOC}
##)
#
#qt4_add_resources(
#  qtpropertyeditor_RESOURCES
#  qtpropertybrowser.qrc
#)
#
#QT4_ADD_RESOURCES(RESOURCE_SRCS qtpropertybrowser.qrc)
#
#set(SRCS ${extension_MOC}
#         ${qtpropertyeditor_MOC}
#         ${qtpropertyeditor_SRCS}
#         ${qtpropertyeditor_RESOURCES}
#)
#
#add_library(gazebo_qtpropertybrowser SHARED ${SRCS})
#
#target_link_libraries(gazebo_qtpropertybrowser ${QT_LIBRARIES})
#
#add_definitions(${QT_DEFINITIONS})
