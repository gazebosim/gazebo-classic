/*
 * Copyright 2012 Open Source Robotics Foundation
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
/* Desc: Trimesh geometry
 * Author: Nate Koenig, Andrew Howard
 * Date: 8 May 2003
 */

#ifndef _TRIMESHSHAPE_HH_
#define _TRIMESHSHAPE_HH_

#include <string>

#include "gazebo/common/CommonTypes.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/Shape.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class TrimeshShape TrimeshShape.hh physics/physics.hh
    /// \brief Triangle mesh collision shape
    class TrimeshShape : public Shape
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent collision.
      public: explicit TrimeshShape(CollisionPtr _parent);

      /// \brief Destructor.
      public: virtual ~TrimeshShape();

      /// \brief Update the tri mesh.
      public: virtual void Update() {}

      /// \copydoc Shape::Init()
      public: virtual void Init();

      /// \brief Get the size of the triangle mesh.
      /// \return The size of the triangle mesh.
      public: virtual math::Vector3 GetSize() const;

      /// \brief Get the filename of the mesh data.
      /// \return The filename of the mesh data.
      public: std::string GetFilename() const;

      /// \brief Set the filename of the triangle mesh.
      /// \param[in] _filename Filename of the mesh file to load from.
      public: void SetFilename(const std::string &_filename);

      /// \brief Set the scaling factor.
      /// \param[in] _scale Scaling factor.
      public: void SetScale(const math::Vector3 &_scale);

      /// \brief Populate a msgs::Geometry message with data from this
      /// shape.
      /// \param[out] _msg Message to fill.
      public: void FillMsg(msgs::Geometry &_msg);

      /// \brief Update this shape from a message.
      /// \param[in] _msg Message that contains triangle mesh info.
      public: virtual void ProcessMsg(const msgs::Geometry &_msg);

      /// \brief Pointer to the mesh data.
      protected: const common::Mesh *mesh;
    };
    /// \}
  }
}
#endif
