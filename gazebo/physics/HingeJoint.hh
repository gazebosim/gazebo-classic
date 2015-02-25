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
/* Desc: A body that has a box shape
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */

#ifndef _HINGEJOINT_HH_
#define _HINGEJOINT_HH_

#include "gazebo/math/Angle.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class HingeJoint HingeJoint.hh physics/physics.hh
    /// \brief A single axis hinge joint
    template<class T>
    class GAZEBO_VISIBLE HingeJoint : public T
    {
      /// \brief Constructor
      /// \param[in] _parent Parent link
      public: HingeJoint(BasePtr _parent) : T(_parent)
              { this->AddType(Base::HINGE_JOINT); }
      ///  \brief Destructor
      public: virtual ~HingeJoint()
              { }

      // Documentation inherited.
      public: virtual unsigned int GetAngleCount() const
              {return 1;}

      /// \brief Load joint
      /// \param[in] _sdf Pointer to SDF element
      public: virtual void Load(sdf::ElementPtr _sdf)
              {
                T::Load(_sdf);
              }

      /// \brief Initialize joint
      protected: virtual void Init()
                 {
                   T::Init();
                 }
    };
    /// \}
  }
}
#endif

