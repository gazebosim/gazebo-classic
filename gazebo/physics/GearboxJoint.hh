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
/* Desc: A body that has a box shape
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */

#ifndef _GEARBOXJOINT_HH_
#define _GEARBOXJOINT_HH_

#include "gazebo/math/Angle.hh"
#include "gazebo/math/Vector3.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class GearboxJoint GearboxJoint.hh physics/physics.hh
    /// \brief A single axis hinge joint
    template<class T>
    class GearboxJoint : public T
    {
      /// \brief Constructor
      /// \param[in] _parent Parent link
      public: GearboxJoint(BasePtr _parent) : T(_parent)
              { this->AddType(Base::HINGE_JOINT); }
      ///  \brief Destructor
      public: virtual ~GearboxJoint()
              { }

      /// \interal
      public: virtual unsigned int GetAngleCount() const
              {return 1;}

      /// \brief Load joint
      /// \param[in] _sdf Pointer to SDF element
      public: virtual void Load(sdf::ElementPtr _sdf)
              {
                T::Load(_sdf);

                this->SetAxis(0,
                    _sdf->GetElement("axis")->GetValueVector3("xyz"));

                this->SetAxis(1,
                    _sdf->GetElement("axis2")->GetValueVector3("xyz"));
              }

      /// \brief Initialize joint
      // protected: virtual void Init()
      //            {
      //              T::Init();
      //            }
    };
    /// \}
  }
}
#endif

