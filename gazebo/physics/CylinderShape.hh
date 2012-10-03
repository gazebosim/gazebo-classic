/*
 * Copyright 2011 Nate Koenig
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

#ifndef CYLINDERSHAPE_HH
#define CYLINDERSHAPE_HH

#include "physics/Shape.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \brief Cylinder collision
    class CylinderShape : public Shape
    {
      /// \brief Constructor
      public: CylinderShape(CollisionPtr parent);

      /// \brief Destructor
      public: virtual ~CylinderShape();

      /// \brief Initialize the cylinder
      public: void Init();

      /// \brief Set radius
      public: void SetRadius(const double &radius);

      /// \brief Set length
      public: void SetLength(const double &length);

       /// \brief Get radius
      public: double GetRadius() const;

      /// \brief Get length
      public: double GetLength() const;

      /// \brief Set the size of the cylinder
      public: virtual void SetSize(const double &radius, const double &length);

      public: void FillShapeMsg(msgs::Geometry &_msg);

      public: virtual void ProcessMsg(const msgs::Geometry &_msg);

      public: virtual double GetMass(double _density) const;

      /// \brief Get inertial for a shape
      public: virtual void GetInertial(double _mass,
                                       InertialPtr _inertial) const;
    };
    /// \}
  }
}
#endif
