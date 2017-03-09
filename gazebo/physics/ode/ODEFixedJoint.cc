/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
  gzwarn << "SimbodyFixedJoint: called method "
         << "GetAnchor that is not valid for joints of type fixed.\n";
  return math::Vector3();
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
  gzwarn << "SimbodyFixedJoint: called method "
         << "GetGlobalAxis that is not valid for joints of type fixed.\n";
  return math::Vector3();
}

//////////////////////////////////////////////////
void ODEFixedJoint::SetAxis(unsigned int /*_index*/,
                            const math::Vector3 &/*_axis*/)
{
  gzwarn << "ODEFixedJoint: called method "
         << "SetAxis that is not valid for joints of type fixed.\n";
}

//////////////////////////////////////////////////
math::Angle ODEFixedJoint::GetAngleImpl(unsigned int /*index*/) const
{
  gzwarn << "ODEFixedJoint: called method "
         << "GetAngleImpl that is not valid for joints of type fixed.\n";
  return math::Angle();
}

//////////////////////////////////////////////////
double ODEFixedJoint::GetVelocity(unsigned int /*index*/) const
{
  gzwarn << "ODEFixedJoint: called method "
         << "GetVelocity that is not valid for joints of type fixed.\n";
  return 0.0;
}

//////////////////////////////////////////////////
void ODEFixedJoint::SetVelocity(unsigned int /*_index*/, double /*_angle*/)
{
  gzwarn << "ODEFixedJoint: called method "
         << "SetVelocity that is not valid for joints of type fixed.\n";
}

//////////////////////////////////////////////////
void ODEFixedJoint::SetForceImpl(unsigned int /*_index*/, double /*_effort*/)
{
  gzwarn << "ODEFixedJoint: called method "
         << "SetForceImpl that is not valid for joints of type fixed.\n";
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
