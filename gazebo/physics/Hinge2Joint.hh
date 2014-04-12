/*
 * Copyright (C) 2012-2013 Open Source Robotics Foundation
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
#ifndef _HINGE2JOINT_HH_
#define _HINGE2JOINT_HH_

#include <sdf/sdf.hh>

#include "gazebo/math/Angle.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/physics/Joint.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class Hinge2Joint Hinge2Joint.hh physics/physics.hh
    /// \brief A two axis hinge joint
    template< class T>
    class Hinge2Joint : public T
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent link.
      public: explicit Hinge2Joint(BasePtr _parent) : T(_parent)
              {this->AddType(Base::HINGE2_JOINT);}

      /// \brief Destructor.
      public: virtual ~Hinge2Joint()
              { }

      // Documentation inherited.
      public: virtual unsigned int GetAngleCount() const
              {return 2;}

      /// \brief Load the joint.
      /// \param[in] _sdf SDF values to load from.
      public: virtual void Load(sdf::ElementPtr _sdf) GAZEBO_DEPRECATED(3.0)
              {
                rml::Joint rmlJoint;
                rmlJoint.SetFromXML(_sdf);
                T::Load(rmlJoint);

                this->SetAxis(0,
                    _sdf->GetElement("axis")->Get<math::Vector3>("xyz"));

                this->SetAxis(1,
                    _sdf->GetElement("axis2")->Get<math::Vector3>("xyz"));
              }

      /// \brief Load the joint.
      /// \param[in] _rml RML values to load from.
      /// \return True on success
      public: virtual bool Load(const rml::Joint &_rml)
              {
                bool result = T::Load(_rml);
                
                this->SetAxis(0, math::Vector3(
                    _rml.axis().xyz().x,
                    _rml.axis().xyz().y,
                    _rml.axis().xyz().z));

                this->SetAxis(1, math::Vector3(
                    _rml.axis2().xyz().x,
                    _rml.axis2().xyz().y,
                    _rml.axis2().xyz().z));

                return result;
              }
    };
    /// \}
  }
}
#endif
