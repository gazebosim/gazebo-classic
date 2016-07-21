/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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
#ifndef GAZEBO_PHYSICS_FIXEDJOINT_HH_
#define GAZEBO_PHYSICS_FIXEDJOINT_HH_

#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class FixedJoint FixedJoint.hh physics/physics.hh
    /// \brief A fixed joint rigidly connecting two bodies
    template<class T>
    class GZ_PHYSICS_VISIBLE FixedJoint : public T
    {
      /// \brief Constructor
      /// \param[in] _parent Parent link
      public: explicit FixedJoint(BasePtr _parent) : T(_parent)
      {
        this->AddType(Base::FIXED_JOINT);
      }

      /// \brief Destructor
      public: virtual ~FixedJoint()
      {
      }

      // Documentation inherited.
      public: virtual unsigned int AngleCount() const
      {
        return 0;
      }

      // Documentation inherited.
      public: virtual void Load(sdf::ElementPtr _sdf)
      {
        T::Load(_sdf);
      }

      // Documentation inherited.
      protected: virtual void Init()
      {
        T::Init();
      }
    };
    /// \}
  }
}
#endif

