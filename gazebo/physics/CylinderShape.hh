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
/* Desc: Cylinder geometry
 * Author: Nate Koenig, Andrew Howard
 * Date: 8 May 2003
 */

#ifndef _CYLINDERSHAPE_HH_
#define _CYLINDERSHAPE_HH_

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include "gazebo/physics/Shape.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class CylinderShape CylinderShape.hh physics/physics.hh
    /// \brief Cylinder collision
    class GZ_PHYSICS_VISIBLE CylinderShape : public Shape
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent of the shape.
      public: explicit CylinderShape(CollisionPtr _parent);

      /// \brief Destructor.
      public: virtual ~CylinderShape();

      /// \brief Initialize the cylinder.
      public: void Init();

      /// \brief Set radius.
      /// \param[in} _radius New radius of the cylinder.
      public: void SetRadius(double _radius);

      /// \brief Set length.
      /// \param[in] _length New length of the cylinder.
      public: void SetLength(double _length);

      /// \brief Get radius.
      /// \return The cylinder radius.
      public: double GetRadius() const;

      /// \brief Get length.
      /// \return The cylinder length.
      public: double GetLength() const;

      /// \brief Set the size of the cylinder.
      /// \param[in] _radius New radius.
      /// \param[in] _lenght New length.
      public: virtual void SetSize(double _radius, double _length);

      /// \brief Set scale of cylinder.
      /// \param[in] _scale Scale to set the cylinder to.
      public: virtual void SetScale(const math::Vector3 &_scale);

      /// \brief Fill in the values for a geomertry message.
      /// \param[out] _msg The geometry message to fill.
      public: void FillMsg(msgs::Geometry &_msg);

      /// \brief Update values based on a message.
      /// \param[in] _msg Message to update from.
      public: virtual void ProcessMsg(const msgs::Geometry &_msg);

      /// Documentation inherited
      public: virtual double ComputeVolume() const;
    };
    /// \}
  }
}
#endif
