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
/* Desc: Box geometry
 * Author: Nate Koenig, Andrew Howard
 * Date: 8 May 2003
 */

#ifndef _BOXSHAPE_HH_
#define _BOXSHAPE_HH_

#include "gazebo/physics/Shape.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class BoxShape BoxShape.hh physics/physcs.hh
    /// \brief Box geometry primitive.
    class BoxShape : public Shape
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent Collision.
      public: explicit BoxShape(CollisionPtr _parent);

      /// \brief Destructor.
      public: virtual ~BoxShape();

      /// \brief Initialize the box.
      public: virtual void Init();

      /// \brief Set the size of the box.
      /// \param[in] _size Size of each side of the box.
      public: virtual void SetSize(const math::Vector3 &_size);

      /// \brief Set the scale of the box.
      /// \param[in] _scale Scale of the box.
      public: virtual void SetScale(const math::Vector3 &_scale);

      /// \brief Get the size of the box.
      /// \return The size of each side of the box.
      public: math::Vector3 GetSize() const;

      /// \brief Fill in the values for a geomertry message.
      /// \param[out] _msg The geometry message to fill.
      public: void FillMsg(msgs::Geometry &_msg);

      /// \brief Process a geometry message.
      /// \param[in] _msg The message to set values from.
      public: virtual void ProcessMsg(const msgs::Geometry &_msg);
    };
    /// \}
  }
}
#endif
