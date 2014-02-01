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
/* Desc: A ball joint
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */

#ifndef _BALLJOINT_HH_
#define _BALLJOINT_HH_

#include "gazebo/physics/Joint.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class BallJoint BallJoint.hh physics/physics.hh
    /// \brief Base class for a ball joint
    ///
    /// Each physics engine should implement this class.
    template< class T>
    class BallJoint : public T
    {
      /// \brief Constructor
      /// \param[in] _parent Pointer to the parent link.
      public: explicit BallJoint(BasePtr _parent) : T(_parent)
              {
                this->AddType(Base::BALL_JOINT);
              }

      /// \brief Destructor
      public: virtual ~BallJoint()
              {
              }

      /// \brief Template to ::Load the BallJoint.
      /// \param[in] _sdf SDF to load the joint from.
      public: void Load(sdf::ElementPtr _sdf)
              {T::Load(_sdf);}

      /// \internal
      /// \brief Set the axis of rotation. This is not used for ball joints.
      public: virtual void SetAxis(int /*_index*/,
                                   const math::Vector3 &/*_axis*/) {}

      /// \internal
      /// \brief Set the high stop of an axis(index).
      public: virtual void SetHighStop(int /*_index*/,
                                       math::Angle /*_angle*/) {}

      /// \internal
      /// \brief Set the low stop of an axis(index).
      public: virtual void SetLowStop(int /*_index*/, math::Angle /*_angle*/) {}

      /// \internal
      /// \brief Get the high stop of an axis(index).
      public: virtual math::Angle GetHighStop(int /*_index*/)
              {return math::Angle();}

      /// \internal
      /// \brief Get the low stop of an axis(index).
      public: virtual math::Angle GetLowStop(int /*_index*/)
              {return math::Angle();}

      /// \internal
      public: virtual unsigned int GetAngleCount() const
              {return 0;}
    };
    /// \}
  }
}
#endif
