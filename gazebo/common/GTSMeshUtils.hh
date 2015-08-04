/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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

#include <ignition/math/Vector2.hh>

#include "gazebo/common/Mesh.hh"
#include "gazebo/math/Vector2d.hh"
#include "gazebo/math/Vector2i.hh"

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
    class GZ_COMMON_VISIBLE GTSMeshUtils
    {
      /// \brief Perform delaunay triangulation on input vertices.
      /// \param[in] _vertices A list of all vertices
      /// \param[in] _edges A list of edges. Each edge is made of 2 vertex
      /// indices from _vertices
      /// \param[out] _submesh A submesh that will be populated with the
      /// resulting triangles.
      /// \return True on success.
      /// \deprecated See DelaunayTriangulation function that accepts
      /// ignition::math objects.
      public: static bool DelaunayTriangulation(
                  const std::vector<math::Vector2d> &_vertices,
                  const std::vector<math::Vector2i> &_edges,
                  SubMesh *_submesh) GAZEBO_DEPRECATED(6.0);

      /// \brief Perform delaunay triangulation on input vertices.
      /// \param[in] _vertices A list of all vertices
      /// \param[in] _edges A list of edges. Each edge is made of 2 vertex
      /// indices from _vertices
      /// \param[out] _submesh A submesh that will be populated with the
      /// resulting triangles.
      /// \return True on success.
      public: static bool DelaunayTriangulation(
                  const std::vector<ignition::math::Vector2d> &_vertices,
                  const std::vector<ignition::math::Vector2i> &_edges,
                  SubMesh *_submesh);


      /// \brief Perform delaunay triangulation on input vertices.
      /// \param[in] _vertices A list of all vertices
      /// \param[in] _edges A list of edges. Each edge is made of 2 vertex
      /// indices from _vertices
      /// \return Triangulated GTS surface.
      private: static GtsSurface *DelaunayTriangulation(
                   const std::vector<ignition::math::Vector2d> &_vertices,
                   const std::vector<ignition::math::Vector2i> &_edges);
    };
  }
}
#endif
