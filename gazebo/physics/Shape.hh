/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#ifndef GAZEBO_PHYSICS_SHAPE_HH_
#define GAZEBO_PHYSICS_SHAPE_HH_

#include <string>
#include <ignition/math/Vector3.hh>

#include "gazebo/msgs/msgs.hh"

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/Inertial.hh"
#include "gazebo/physics/Base.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class Shape Shape.hh physics/physics.hh
    /// \brief Base class for all shapes.
    class GZ_PHYSICS_VISIBLE Shape : public Base
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent of the shape.
      public: explicit Shape(CollisionPtr _parent);

      /// \brief Destructor.
      public: virtual ~Shape();

      /// \brief Initialize the shape.
      public: virtual void Init() = 0;

      /// \brief Set the scale of the shape.
      /// \param[in] _scale Scale to set the shape to.
      public: virtual void SetScale(const ignition::math::Vector3d &_scale) = 0;

      /// \brief Get the scale of the shape.
      /// \return Scale of the shape.
      public: virtual ignition::math::Vector3d Scale() const;

      /// \brief Fill in the values for a geometry message.
      /// \param[out] _msg The geometry message to fill.
      public: virtual void FillMsg(msgs::Geometry &_msg) = 0;

      /// \brief Process a geometry message.
      /// \param[in] _msg The message to set values from.
      public: virtual void ProcessMsg(const msgs::Geometry &_msg) = 0;

      /// \brief Get the volume of this shape. Implemented accurately for
      /// simple shapes; an approximation is used for meshes, polylines, etc.
      /// \return The shape volume in kg/m^3.
      public: virtual double ComputeVolume() const;

      /// \brief This shape's collision parent.
      protected: CollisionPtr collisionParent;

      /// \brief This shape's scale;
      protected: ignition::math::Vector3d scale = ignition::math::Vector3d::One;
    };
    /// \}
  }
}
#endif
