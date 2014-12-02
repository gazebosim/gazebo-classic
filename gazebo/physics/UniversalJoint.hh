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
#ifndef _GAZEBO_UNIVERSALJOINT_HH_
#define _GAZEBO_UNIVERSALJOINT_HH_

#include "gazebo/math/Vector3.hh"
#include "gazebo/physics/Joint.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class UniversalJoint UniversalJoint.hh physics/physics.hh
    /// \brief A universal joint.
    /// Axis1 and axis2 are body-fixed, with axis1 attached to parent
    /// body and axis2 attached to child body.
    template<class T>
    class GZ_PHYSICS_VISIBLE UniversalJoint : public T
    {
      /// \enum AxisIndex
      /// \brief Map joint axes to corresponding link.
      public: enum AxisIndex
      {
        AXIS_PARENT = 0,
        AXIS_CHILD  = 1
      };

      /// \brief Constructor.
      /// \param[in] _parent Parent link of the univeral joint.
      public: explicit UniversalJoint(BasePtr _parent) : T(_parent)
              {this->AddType(Base::UNIVERSAL_JOINT);}

      /// \brief Destuctor.
      public: virtual ~UniversalJoint()
              { }

      // Documentation inherited.
      public: virtual unsigned int GetAngleCount() const
              {return 2;}

      /// \brief Load a UniversalJoint.
      /// \param[in] _sdf SDF values to load from.
      public: virtual void Load(sdf::ElementPtr _sdf)
              {
                T::Load(_sdf);

                /*
                this->SetAxis(0,
                    this->sdf->GetElement("axis")->Get<math::Vector3("xyz"));
                this->SetAxis(1,
                    this->sdf->GetElement("axis2")->Get<math::Vector3>("xyz"));
                    */
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
