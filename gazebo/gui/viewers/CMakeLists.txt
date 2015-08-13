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

set (headers ${headers}
  viewers/ViewFactory.hh
  PARENT_SCOPE
)

set (qt_headers ${qt_headers}
  viewers/ImageFrame.hh
  viewers/ImageView.hh
  viewers/ImagesView.hh
  viewers/LaserView.hh
  viewers/TextView.hh
  viewers/TopicView.hh
  PARENT_SCOPE
)

set (qt_tests
  ImagesView_TEST.cc
)
