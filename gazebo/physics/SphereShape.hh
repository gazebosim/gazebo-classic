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
/* Desc: Sphere shape
 * Author: Nate Koenig
 * Date: 14 Oct 2009
 */

#ifndef SPHERESHAPE_HH
#define SPHERESHAPE_HH

#include "physics/Shape.hh"
#include "physics/PhysicsTypes.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \brief Sphere collision
    class SphereShape : public Shape
    {
      /// \brief Constructor
      public: SphereShape(CollisionPtr parent);

      /// \brief Destructor
      public: virtual ~SphereShape();

      /// \brief Initialize the sphere
      public: virtual void Init();

      /// \brief Set the size
      public: virtual void SetRadius(const double &radius);

      public: double GetRadius() const;

      public: virtual void FillShapeMsg(msgs::Geometry &_msg);

      public: virtual void ProcessMsg(const msgs::Geometry &_msg);

      /// \brief Get the mass of a shape
      public: virtual double GetMass(double _density) const;

      /// \brief Get inertial for a shape
      public: virtual void GetInertial(double _mass,
                                       InertialPtr _inertial) const;
    };
    /// \}
  }
}
#endif


