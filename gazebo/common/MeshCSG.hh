/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#ifndef _MESHCSG_HH_
#define _MESHCSG_HH_

#include "gazebo/math/Pose.hh"

struct _GtsSurface;
typedef _GtsSurface GtsSurface;
struct _GPtrArray;
typedef _GPtrArray GPtrArray;

namespace gazebo
{
  namespace common
  {
    class Mesh;

    /// \addtogroup gazebo_common Common
    /// \{

    /// \class MeshCSG MeshCSG.hh common/common.hh
    /// \brief Creates CSG meshes
    class MeshCSG
    {
      /// \brief An enumeration of the boolean operations
      public: enum BooleanOperation {UNION, INTERSECTION, DIFFERENCE};

      /// \brief Constructor
      public: MeshCSG();

      /// \brief Destructor.
      public: virtual ~MeshCSG();

      /// \brief Create a boolean mesh from two meshes
      /// \param[in] _m1 the parent mesh in the boolean operation
      /// \param[in] _m2 the child mesh in the boolean operation
      /// \param[in] _operation the boolean operation applied to the two meshes
      /// \param[in] _offset _m2's pose offset from _m1
      /// \return a pointer to the created mesh
      public: Mesh *CreateBoolean(const Mesh *_m1, const Mesh *_m2,
          const int _operation, const math::Pose &_offset = math::Pose::Zero);

      /// \brief Helper method for converting Mesh to GTS Surface
      private: void ConvertMeshToGTS(const Mesh *mesh, GtsSurface *surface);

      /// \brief Helper method for merging neighborhood vertices whose positions
      // are within epsilon
      /// \param[in] _vertices Array of GTS vertices.
      /// \param[in] _epsilon Epsilon
      private: void MergeVertices(GPtrArray * _vertices, double _epsilon);
    };
    /// \}
  }
}

#endif
