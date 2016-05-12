/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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

#ifndef _GAZEBO_ODEMESH_HH_
#define _GAZEBO_ODEMESH_HH_

#include "gazebo/physics/ode/ODETypes.hh"
#include "gazebo/physics/ode/ode_inc.h"
#include "gazebo/physics/MeshShape.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics_ode
    /// \{

    /// \brief Triangle mesh helper class.
    class GZ_PHYSICS_VISIBLE ODEMesh
    {
      /// \brief Constructor.
      public: explicit ODEMesh();

      /// \brief Destructor.
      public: virtual ~ODEMesh();

      /// \brief Create a mesh collision shape using a submesh.
      /// \param[in] _subMesh Pointer to the submesh.
      /// \param[in] _collision Pointer to the collsion object.
      /// \param[in] _scale Scaling factor.
      public: void Init(const common::SubMesh *_subMesh,
                  ODECollisionPtr _collision,
                  const math::Vector3 &_scale);

      /// \brief Create a mesh collision shape using a mesh.
      /// \param[in] _mesh Pointer to the mesh.
      /// \param[in] _collision Pointer to the collsion object.
      /// \param[in] _scale Scaling factor.
      public: void Init(const common::Mesh *_mesh, ODECollisionPtr _collision,
                  const math::Vector3 &_scale);

      /// \brief Update the collision mesh.
      public: virtual void Update();

      /// \brief Helper function to create the collision shape.
      /// \param[in] _numVertices Number of vertices.
      /// \param[in] _numIndices Number of indices.
      /// \param[in] _collision Pointer to the collsion object.
      private: void CreateMesh(unsigned int _numVertices,
                   unsigned int _numIndices, ODECollisionPtr _collision,
                   const math::Vector3 &_scale);

      /// \brief Transform matrix.
      private: dReal transform[16*2];

      /// \brief Transform matrix index.
      private: int transformIndex;

      /// \brief Array of vertex values.
      private: float *vertices;

      /// \brief Array of index values.
      private: int *indices;

      /// \brief ODE trimesh data.
      private: dTriMeshDataID odeData;

      /// \brief The collision id that this mesh is attached to.
      private: dGeomID collisionId;
    };
    /// \}
  }
}
#endif
