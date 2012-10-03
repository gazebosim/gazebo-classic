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
/* Desc: Box geometry
 * Author: Nate Koenig, Andrew Howard
 * Date: 8 May 2003
 */

#ifndef BOXSHAPE_HH
#define BOXSHAPE_HH

#include "physics/Shape.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \brief Box geometry primitive
    class BoxShape : public Shape
    {
      /// \brief Constructor
      public: BoxShape(CollisionPtr parent);

      /// \brief Destructor
      public: virtual ~BoxShape();

      /// \brief Initialize the box
      public: virtual void Init();

      /// \brief Set the size of the box
      public: virtual void SetSize(const math::Vector3 &size);

      /// \brief Get the size of the box
      public: math::Vector3 GetSize() const;

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
