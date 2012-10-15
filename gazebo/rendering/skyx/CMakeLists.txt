# define header and source files for the library
set (SKYX_HEADER_FILES
  include/AtmosphereManager.h
  include/BasicController.h
  include/CloudsManager.h
  include/ColorGradient.h
  include/Controller.h
  include/GPUManager.h
  include/MeshManager.h
  include/MoonManager.h
  include/Prerequisites.h
  include/SkyX.h
  include/VCloudsManager.h
)

set (SKYX_SOURCE_FILES
  src/AtmosphereManager.cpp
  src/BasicController.cpp
  src/CloudsManager.cpp
  src/ColorGradient.cpp
  src/GPUManager.cpp
  src/MeshManager.cpp
  src/MoonManager.cpp
  src/SkyX.cpp
  src/VCloudsManager.cpp
)

set (VCLOUDS_HEADER_FILES
  include/VClouds/DataManager.h
  include/VClouds/Ellipsoid.h
  include/VClouds/FastFakeRandom.h
  include/VClouds/GeometryBlock.h
  include/VClouds/GeometryManager.h
  include/VClouds/Lightning.h
  include/VClouds/LightningManager.h
  include/VClouds/VClouds.h
)

set (VCLOUDS_SOURCE_FILES
  src/VClouds/DataManager.cpp
  src/VClouds/Ellipsoid.cpp
  src/VClouds/FastFakeRandom.cpp
  src/VClouds/GeometryBlock.cpp
  src/VClouds/GeometryManager.cpp
  src/VClouds/Lightning.cpp
  src/VClouds/LightningManager.cpp
  src/VClouds/VClouds.cpp
)

set (CMAKE_C_FLAGS "")
set (CMAKE_CXX_FLAGS "")

#set_source_files_properties(${SKYX_SOURCE_FILES} ${SKYX_HEADER_FILES}
#                      ${VCLOUDS_SOURCE_FILES} ${VCLOUDS_HEADER_FILES}
#                      PROPERTIES COMPILE_FLAGS
#                      ${CMAKE_CXX_FLAGS_${CMAKE_BUILD_TYPE}_CLEAN})
#
# Add include directory
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

gz_add_library(gazebo_skyx ${SKYX_SOURCE_FILES} ${VCLOUDS_SOURCE_FILES})
target_link_libraries(gazebo_skyx ${OGRE_LIBRARIES})
gz_install_library(gazebo_skyx)
