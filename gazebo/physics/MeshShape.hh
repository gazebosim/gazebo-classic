/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifndef _GAZEBO_PHYSICS_MESHSHAPE_HH_
#define _GAZEBO_PHYSICS_MESHSHAPE_HH_

#include <string>
#include <ignition/math/Vector3.hh>

#include "gazebo/common/CommonTypes.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/Shape.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    // Forward declare private data class
    class MeshShapePrivate;

    /// \addtogroup gazebo_physics
    /// \{

    /// \class MeshShape MeshShape.hh gazebo/physics/MeshShape.hh
    /// \brief Triangle mesh collision shape
    class GZ_PHYSICS_VISIBLE MeshShape : public Shape
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent collision.
      public: explicit MeshShape(CollisionPtr _parent);

      /// \brief Destructor.
      public: virtual ~MeshShape();

      /// \brief Update the tri mesh.
      public: virtual void Update() {}

      /// \copydoc Shape::Init()
      public: virtual void Init();

      /// \brief Get the size of the triangle mesh.
      /// \return The size of the triangle mesh.
      /// \deprecated See function that returns an ignition::math object.
      public: virtual math::Vector3 GetSize() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the size of the triangle mesh.
      /// \return The size of the triangle mesh.
      public: virtual ignition::math::Vector3d Size() const;

      /// \brief Get the URI of the mesh data.
      /// \return The URI of the mesh data.
      /// \deprecated See MeshURI() const
      public: std::string GetMeshURI() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the URI of the mesh data.
      /// \return The URI of the mesh data.
      public: std::string MeshURI() const;

      /// \brief Set the mesh uri and submesh name.
      /// \param[in] _uri Filename of the mesh file to load from.
      /// \param[in] _submesh Name of the submesh to use within the mesh
      /// \param[in] _center True to center the submesh.
      /// specified in the _uri.
      public: void SetMesh(const std::string &_uri,
                           const std::string &_submesh = "",
                           const bool _center = false);


      /// \brief Set the scaling factor.
      /// \param[in] _scale Scaling factor.
      /// \deprecated See function that accepts ignition::math params
      public: void SetScale(const math::Vector3 &_scale) GAZEBO_DEPRECATED(7.0);

      /// \brief Set the scaling factor.
      /// \param[in] _scale Scaling factor.
      public: void SetScale(const ignition::math::Vector3d &_scale);

      /// \brief Populate a msgs::Geometry message with data from this
      /// shape.
      /// \param[out] _msg Message to fill.
      public: void FillMsg(msgs::Geometry &_msg);

      /// \brief Update this shape from a message.
      /// \param[in] _msg Message that contains triangle mesh info.
      public: virtual void ProcessMsg(const msgs::Geometry &_msg);

      /// \internal
      /// \brief Pointer to mesh shape private data
      protected: MeshShapePrivate *meshShapeDPtr;
    };
    /// \}
  }
}
#endif
