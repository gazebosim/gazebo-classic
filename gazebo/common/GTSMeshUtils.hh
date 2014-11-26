/*
 * Copyright (C) 2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef _GAZEBO_GTSMESHUTILS_HH_
#define _GAZEBO_GTSMESHUTILS_HH_

#include <vector>

#include "gazebo/common/Mesh.hh"
#include "gazebo/math/Vector2d.hh"

namespace gazebo
{
  namespace common
  {
    class Mesh;

    /// \addtogroup gazebo_common Common
    /// \{

    /// \class GTSMeshUtils GTSMeshUtils.hh common/common.hh
    /// \brief Creates GTS utilities for meshes
    class GZ_COMMON_VISIBLE GTSMeshUtils
    {
      /// \brief Create an extruded Polyline submesh
      /// \param[in] _vertices the x y dimentions of eah vertex in meter
      /// \param[in] _height the height of the polyline
      /// \param[out] _submesh A submesh that will be populated with the
      /// extruded polyline.
      /// \return True on success.
      public: static bool CreateExtrudedPolyline(
                  const std::vector<math::Vector2d> &_vertices,
                  const double &_height,
                  SubMesh *_submesh);
    };
  }
}
#endif
