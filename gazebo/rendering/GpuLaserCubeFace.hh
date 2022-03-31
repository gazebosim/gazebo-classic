/*
 * Enway GmbH - All Rights reserved.
 * Proprietary & confidential.
 */

#ifndef GAZEBO_GPU_LASER_CUBE_FACE_H
#define GAZEBO_GPU_LASER_CUBE_FACE_H

#include <string>
#include <utility>
#include <vector>

#include <ignition/math/Vector2.hh>

#include "gazebo/rendering/ogre_gazebo.h"

namespace gazebo
{

  namespace rendering
  {

    /// \brief Cube map face ID
    enum class GpuLaserCubeFaceId
    {
      CUBE_FRONT_FACE,
      CUBE_LEFT_FACE,
      CUBE_REAR_FACE,
      CUBE_RIGHT_FACE,
      CUBE_TOP_FACE,
      CUBE_BOTTOM_FACE
    };

    /// \brief Stores mapping of a single laser ray (combination of azimuth and
    /// elevation) to cube map coordinates. The first element of the pair is the
    /// ID of the corresponding cube map face. The second element are the
    /// normalized x/y coordinates of the intersection of the laser ray with
    /// that cube face (in the range [0,1]x[0,1]).
    typedef std::pair<GpuLaserCubeFaceId, ignition::math::Vector2d>
        GpuLaserCubeMappingPoint;

    /// \brief Orientation offset for camera
    struct GpuLaserCameraOrientationOffset
    {
      double azimuthOffset;
      double elevationOffset;
    };

    /// \brief Holds the data for each cube face.
    struct GpuLaserCubeFace
    {
      /// \brief The corresponding camera orientation offset
      GpuLaserCameraOrientationOffset cameraSetting;

      /// \brief The texture used to render the depth image
      Ogre::TexturePtr texture;

      /// \brief The depth image data
      std::vector<float> depthImg;
    };

  }

}

#endif //GAZEBO_GPU_LASER_CUBE_FACE_H
