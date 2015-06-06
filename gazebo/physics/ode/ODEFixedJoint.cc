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

#include <boost/bind.hpp>

#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"

#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/ode/ODEFixedJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
ODEFixedJoint::ODEFixedJoint(dWorldID _worldId, BasePtr _parent)
    : FixedJoint<ODEJoint>(_parent)
{
  this->jointId = dJointCreateFixed(_worldId, NULL);
}

//////////////////////////////////////////////////
ODEFixedJoint::~ODEFixedJoint()
{
}

//////////////////////////////////////////////////
void ODEFixedJoint::Load(sdf::ElementPtr _sdf)
{
  FixedJoint<ODEJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
math::Vector3 ODEFixedJoint::GetAnchor(unsigned int /*index*/) const
{
  dVector3 result;

  gzerr << "invalid method GetAnchor for fixed joints\n";

  return math::Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
void ODEFixedJoint::SetAnchor(unsigned int /*index*/,
    const math::Vector3 &/*_anchor*/)
{
  if (this->childLink)
    this->childLink->SetEnabled(true);
  if (this->parentLink)
    this->parentLink->SetEnabled(true);
}

//////////////////////////////////////////////////
math::Vector3 ODEFixedJoint::GetGlobalAxis(unsigned int /*_index*/) const
{
  dVector3 result;

  gzerr << "called invalid method GetGlobalAxis in a fixed joint\n";

  return math::Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
void ODEFixedJoint::SetAxis(unsigned int /*_index*/, const math::Vector3 &/*_axis*/)
{
  gzerr << "called invalid method SetAxis in a fixed joint\n";
}

//////////////////////////////////////////////////
math::Angle ODEFixedJoint::GetAngleImpl(unsigned int /*index*/) const
{
  math::Angle result;

  gzerr << "called invalid method GetAngleImpl in a fixed joint\n";

  return result;
}

//////////////////////////////////////////////////
double ODEFixedJoint::GetVelocity(unsigned int /*index*/) const
{
  double result = 0;

  gzerr << "called invalid method GetVelocity in a fixed joint\n";

  return result;
}

//////////////////////////////////////////////////
void ODEFixedJoint::SetVelocity(unsigned int /*_index*/, double /*_angle*/)
{
  gzerr << "called invalid method SetVelocity in a fixed joint\n";
}

//////////////////////////////////////////////////
void ODEFixedJoint::SetMaxForce(unsigned int /*index*/, double /*_t*/)
{
  gzerr << "called invalid method SetMaxForce in a fixed joint\n";
}

//////////////////////////////////////////////////
double ODEFixedJoint::GetMaxForce(unsigned int /*index*/)
{
  gzerr << "called invalid method GetMaxForce in a fixed joint\n";
  return 0;
}

//////////////////////////////////////////////////
void ODEFixedJoint::SetForceImpl(unsigned int /*_index*/, double /*_effort*/)
{
  gzerr << "called invalid method SetForceImpl in a fixed joint\n";
}

//////////////////////////////////////////////////
void ODEFixedJoint::Attach(LinkPtr _parent, LinkPtr _child)
{
    gazebo::physics::ODEJoint::Attach(_parent, _child);

    // Once we attach the links to the fixed joint,
    // we also call the dJointSetFixed method to
    // the current desired relative offset and desired
    // relative rotation between the bodies.

    dJointSetFixed(this->jointId);
}
