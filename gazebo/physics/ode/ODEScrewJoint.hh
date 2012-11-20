/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: A screw or primastic joint
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */

#ifndef _ODESCREWJOINT_HH_
#define _ODESCREWJOINT_HH_

#include "gazebo/physics/ScrewJoint.hh"
#include "gazebo/physics/ode/ODEJoint.hh"

namespace gazebo
{
  namespace physics
  {
    /// \brief A screw joint.
    class ODEScrewJoint : public ScrewJoint<ODEJoint>
    {
      /// \brief Constructor.
      /// \param[in] _worldId ODE world id.
      /// \param[in] _parent Pointer to the Link that is the joint' parent
      public: ODEScrewJoint(dWorldID _worldId, BasePtr _parent);

      /// \brief Destructor.
      public: virtual ~ODEScrewJoint();

      // Documentation inherited
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited
      public: virtual math::Vector3 GetGlobalAxis(int _index) const;

      // Documentation inherited
      public: virtual void SetAxis(int index, const math::Vector3 &_axis);

      // Documentation inherited
      public: virtual void SetDamping(int _index, double _damping);

      // Documentation inherited
      public: virtual void SetThreadPitch(int _index, double _threadPitch);

      // Documentation inherited
      public: virtual math::Angle GetAngleImpl(int _index) const;

      // Documentation inherited
      public: virtual double GetVelocity(int _index) const;

      // Documentation inherited
      public: virtual void SetVelocity(int _index, double _angle);

      // Documentation inherited
      public: virtual void SetForce(int _index, double _force);

      // Documentation inherited
      public: virtual void SetMaxForce(int _index, double _t);

      // Documentation inherited
      public: virtual double GetMaxForce(int _index);

      // Documentation inherited
      public: virtual double GetParam(int _parameter) const;

      // Documentation inherited
      public: virtual void SetParam(int _parameter, double _value);

      /// \brief Callback to apply damping force to joint.
      public: void ApplyDamping();
    };
  }
}
#endif
