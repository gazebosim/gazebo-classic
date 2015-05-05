/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
/* Desc: A slider or primastic joint
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */

#ifndef _SLIDERJOINT_HH_
#define _SLIDERJOINT_HH_

#include "gazebo/physics/Joint.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class SliderJoint SliderJoint.hh physics/physics.hh
    /// \brief A slider joint
    template<class T>
    class GZ_PHYSICS_VISIBLE SliderJoint : public T
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent of the joint.
      public: explicit SliderJoint(BasePtr _parent) : T(_parent)
              {this->AddType(Base::SLIDER_JOINT);}

      /// \brief Destructor.
      public: virtual ~SliderJoint()
              {}

      /// \brief Load a SliderJoint.
      /// \param[in] _sdf SDF values to load from
      public: virtual void Load(sdf::ElementPtr _sdf)
              {T::Load(_sdf);}

      // Documentation inherited.
      public: virtual unsigned int GetAngleCount() const
              {return 1;}
    };
    /// \}
  }
}
#endif
