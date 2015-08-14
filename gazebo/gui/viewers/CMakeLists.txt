include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources ${sources}
  viewers/ImageFrame.cc
  viewers/ImageView.cc
  viewers/ImagesView.cc
  viewers/LaserView.cc
  viewers/TopicView.cc
  viewers/TextView.cc
  viewers/ViewFactory.cc
  PARENT_SCOPE
)

set (internal_qt_headers ${internal_qt_headers}
  viewers/ImageFrame.hh
  viewers/ImageView.hh
  viewers/ImagesView.hh
  viewers/LaserView.hh
  viewers/TextView.hh
  viewers/TopicView.hh
  PARENT_SCOPE
)

set (headers_install
  ViewFactory.hh
)

set (qt_tests_local
  ImagesView_TEST.cc
)

gz_build_qt_tests(${qt_tests_local})
gz_install_includes("gui/viewers" ${headers_install})
