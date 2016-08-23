/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifndef _ODEUNIVERSALJOINT_HH_
#define _ODEUNIVERSALJOINT_HH_

#include <boost/any.hpp>
#include <string>

#include "gazebo/common/Exception.hh"
#include "gazebo/physics/UniversalJoint.hh"
#include "gazebo/physics/ode/ODEJoint.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics_ode
    /// \{

    /// \brief A universal joint.
    class GZ_PHYSICS_VISIBLE ODEUniversalJoint :
      public UniversalJoint<ODEJoint>
    {
      /// \brief Constructor.
      /// \param[in] _worldId ODE world id.
      /// \param[in] _parent Pointer to the Link that is the joint' parent
      public: ODEUniversalJoint(dWorldID _worldId, BasePtr _parent);

      /// \brief Destuctor.
      public: virtual ~ODEUniversalJoint();

      // Documentation inherited
      public: virtual math::Vector3 GetAnchor(unsigned int _index) const;

      // Documentation inherited
      public: virtual void SetAnchor(unsigned int _index,
                  const math::Vector3 &_anchor);

      // Documentation inherited
      public: virtual math::Vector3 GetGlobalAxis(unsigned int _index) const;

      // Documentation inherited
      public: virtual void SetAxis(unsigned int _index,
                  const math::Vector3 &_axis);

      // Documentation inherited
      public: virtual math::Angle GetAngleImpl(unsigned int _index) const;

      // Documentation inherited
      public: virtual double GetVelocity(unsigned int _index) const;

      // Documentation inherited
      public: virtual void SetVelocity(unsigned int _index, double _angle);

      // Documentation inherited
      public: virtual void SetParam(unsigned int _parameter, double _value);

      // Documentation inherited.
      public: virtual bool SetHighStop(unsigned int _index,
                  const math::Angle &_angle);

      // Documentation inherited.
      public: virtual bool SetLowStop(unsigned int _index,
                  const math::Angle &_angle);

      // Documentation inherited.
      public: virtual bool SetParam(const std::string &_key,
                                    unsigned int _index,
                                    const boost::any &_value);

      // Documentation inherited.
      public: virtual double GetParam(const std::string &_key,
                                      unsigned int _index);

      // Documentation inherited
      public: virtual double GetParam(unsigned int _parameter) const;

      // Documentation inherited
      protected: virtual void SetForceImpl(unsigned int _index, double _effort);
    };
    /// \}
  }
}
#endif
