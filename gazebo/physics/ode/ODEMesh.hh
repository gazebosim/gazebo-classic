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
#ifndef GAZEBO_PHYSICS_ODE_ODEMESH_HH_
#define GAZEBO_PHYSICS_ODE_ODEMESH_HH_

#include "gazebo/physics/ode/ODETypes.hh"
#include "gazebo/physics/ode/ode_inc.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics_ode
    /// \{

    // Forward declare private data
    class ODEMeshPrivate;

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
      /// \deprecated See function that accepts ignition::math parameters.
      public: void Init(const common::SubMesh *_subMesh,
                  ODECollisionPtr _collision,
                  const math::Vector3 &_scale) GAZEBO_DEPRECATED(7.0);

      /// \brief Create a mesh collision shape using a submesh.
      /// \param[in] _subMesh Pointer to the submesh.
      /// \param[in] _collision Pointer to the collsion object.
      /// \param[in] _scale Scaling factor.
      public: void Init(const common::SubMesh *_subMesh,
                  ODECollisionPtr _collision,
                  const ignition::math::Vector3d &_scale);

      /// \brief Create a mesh collision shape using a mesh.
      /// \param[in] _mesh Pointer to the mesh.
      /// \param[in] _collision Pointer to the collsion object.
      /// \param[in] _scale Scaling factor.
      /// \deprecated See function that accepts ignition::math parameters.
      public: void Init(const common::Mesh *_mesh, ODECollisionPtr _collision,
                  const math::Vector3 &_scale) GAZEBO_DEPRECATED(7.0);

      /// \brief Create a mesh collision shape using a mesh.
      /// \param[in] _mesh Pointer to the mesh.
      /// \param[in] _collision Pointer to the collsion object.
      /// \param[in] _scale Scaling factor.
      public: void Init(const common::Mesh *_mesh, ODECollisionPtr _collision,
                  const ignition::math::Vector3d &_scale);

      /// \brief Update the collision mesh.
      public: virtual void Update();

      /// \brief Helper function to create the collision shape.
      /// \param[in] _numVertices Number of vertices.
      /// \param[in] _numIndices Number of indices.
      /// \param[in] _collision Pointer to the collsion object.
      private: void CreateMesh(const unsigned int _numVertices,
                   const unsigned int _numIndices,
                   ODECollisionPtr _collision,
                   const ignition::math::Vector3d &_scale);

      /// \internal
      /// \brief Private data pointer
      protected: ODEMeshPrivate *odeMeshDPtr;
    };
    /// \}
  }
}
#endif
