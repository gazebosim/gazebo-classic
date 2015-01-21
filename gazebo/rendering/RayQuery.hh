/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef _GAZEBO_RAYQUERY_HH_
#define _GAZEBO_RAYQUERY_HH_

#include <vector>

#include "gazebo/math/Vector3.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  /// \ingroup gazebo_rendering
  /// \brief Rendering namespace
  namespace rendering
  {
    class RayQueryPrivate;

    /// \addtogroup gazebo_rendering
    /// \{

    /// \class RayQuery RayQuery.hh rendering/rendering.hh
    /// \brief A Ray Query class used for retrieving mesh data of a visual,
    /// adapted from Ogre3D wiki.
    class GAZEBO_VISIBLE RayQuery
    {
      /// \brief Constructor
      /// \param[in] _camera Pointer to camera used for ray casting.
      public: RayQuery(CameraPtr _camera);

      /// \brief Destructor
      public: ~RayQuery();

      /// \brief Select a triangle on mesh given screen coordinates
      /// \param[in] _x X position on screen in pixels.
      /// \param[in] _y Y position on screen in pixels.
      /// \param[in] _visual Visual containing the mesh to be selected.
      /// \param[out] _intersect Intersection point.
      /// \param[out] _vertices Vertices of the selected triangle on the mesh.
      public: bool SelectMeshTriangle(int _x, int _y, VisualPtr _visual,
          math::Vector3 &_intersect, std::vector<math::Vector3> &_vertices);

      /// \brief Helper method to recursively find all visuals that have a mesh.
      /// \param[in] _visual Parent visual to be traversed.
      /// \param[out] _visuals A list of visuals with mesh.
      private: void GetMeshVisuals(rendering::VisualPtr _visual,
          std::vector<rendering::VisualPtr> &_visuals);

      /// \internal
      /// \brief Pointer to private data.
      private: RayQueryPrivate *dataPtr;
    };
    /// \}
  }
}
#endif
