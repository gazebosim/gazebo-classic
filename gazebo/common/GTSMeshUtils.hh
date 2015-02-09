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

struct _GtsSurface;
typedef _GtsSurface GtsSurface;


namespace gazebo
{
  namespace common
  {
    class Mesh;

    /// \addtogroup gazebo_common Common
    /// \{

    /// \class GTSMeshUtils GTSMeshUtils.hh common/common.hh
    /// \brief Creates GTS utilities for meshes
    class GAZEBO_VISIBLE GTSMeshUtils
    {
      /// \brief Perform delaunay triangulation on input vertices.
      /// \param[in] _path A path can contain multiple subpath, which in turn
      /// is composed of a list of vertices.
      /// \param[out] _submesh A submesh that will be populated with the
      /// resulting triangles.
      /// \return True on success.
      public: static bool DelaunayTriangulation(
                  const std::vector<std::vector<math::Vector2d> > &_path,
                  SubMesh *_submesh);

      /// \brief Perform delaunay triangulation on input vertices.
      /// \param[in] _path A path can contain multiple subpath, which in turn
      /// is composed of a list of vertices.
      /// \param[out] _submesh A submesh that will be populated with the
      /// resulting triangles.
      /// \return Triangulated GTS surface.
      private: static GtsSurface *DelaunayTriangulation(
                  const std::vector<std::vector<math::Vector2d> > &_path);
    };
  }
}
#endif
