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
/* Desc: A body that has a box shape
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */

#ifndef HINGEJOINT_HH
#define HINGEJOINT_HH

#include "math/Angle.hh"
#include "math/Vector3.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \brief A single axis hinge joint
    template<class T>
    class HingeJoint : public T
    {
      /// \brief Constructor
      public: HingeJoint(BasePtr _parent) : T(_parent)
              { this->AddType(Base::HINGE_JOINT); }
      ///  \brief Destructor
      public: virtual ~HingeJoint()
              { }
      /// \brief Load joint
      protected: virtual void Load(sdf::ElementPtr _sdf)
                 {
                   T::Load(_sdf);
                 }

      protected: virtual void Init()
                 {
                   T::Init();
                 }
    };
    /// \}
  }
}
#endif

