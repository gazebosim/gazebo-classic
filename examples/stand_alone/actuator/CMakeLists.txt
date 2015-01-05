cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

include(FindBoost)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)

find_package(Protobuf REQUIRED)

find_package(SDFormat REQUIRED)

include_directories(${SDFormat_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${SDFormat_LIBRARY_DIRS} ${GAZEBO_LIBRARY_DIRS})

add_executable(actuator main.cc)
target_link_libraries(actuator ${SDFormat_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} pthread)

set (MODEL_INSTALL_PREFIX ~/.gazebo)

set (model_dirs
  actuator_example
  unactuated_example
)

set (MODEL_URI_LIST "" CACHE INTERNAL "Model URI List" FORCE)
get_filename_component(MODEL_INSTALL_PREFIX_ABSOLUTE ${MODEL_INSTALL_PREFIX}
  ABSOLUTE)

foreach (model_dir ${model_dirs})
  #install (DIRECTORY ${model_dir} DESTINATION ${MODEL_INSTALL_PREFIX_ABSOLUTE}/models)
  install (FILES ${model_dir}/model.config ${model_dir}/model.sdf DESTINATION ${MODEL_INSTALL_PREFIX_ABSOLUTE}/models/${model_dir})
  set(MODEL_URI_LIST "${MODEL_URI_LIST}<uri>file://${model_dir}</uri>\n")
endforeach()
