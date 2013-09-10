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

#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"

#include "gazebo/physics/Link.hh"
#include "gazebo/physics/dart/DARTHinge2Joint.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
DARTHinge2Joint::DARTHinge2Joint(BasePtr _parent)
    : Hinge2Joint<DARTJoint>(_parent)
{
}

//////////////////////////////////////////////////
DARTHinge2Joint::~DARTHinge2Joint()
{
}

//////////////////////////////////////////////////
void DARTHinge2Joint::Load(sdf::ElementPtr _sdf)
{
  Hinge2Joint<DARTJoint>::Load(_sdf);
}

void DARTHinge2Joint::Init()
{

}

//////////////////////////////////////////////////
math::Vector3 DARTHinge2Joint::GetAnchor(int /*_index*/) const
{
  gzwarn << "Not implemented!\n";

  return math::Vector3(0, 0, 0);
}

//////////////////////////////////////////////////
void DARTHinge2Joint::SetAnchor(int /*index*/, const math::Vector3 &/*_anchor*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTHinge2Joint::SetAxis(int /*_index*/, const math::Vector3 &/*_axis*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTHinge2Joint::SetDamping(int /*_index*/, double /*_damping*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
math::Vector3 DARTHinge2Joint::GetGlobalAxis(int /*_index*/) const
{
  gzwarn << "Not implemented!\n";

  return math::Vector3(0, 0, 0);
}

//////////////////////////////////////////////////
math::Angle DARTHinge2Joint::GetAngleImpl(int _index) const
{
  math::Angle result;

  if (_index == 0)
    result = this->dartJoint->getGenCoord(0)->get_q();
  else
    result = this->dartJoint->getGenCoord(1)->get_q();

  return result;
}

//////////////////////////////////////////////////
double DARTHinge2Joint::GetVelocity(int /*_index*/) const
{
  double result = 0;

  gzwarn << "Not implemented!\n";

  return result;
}

//////////////////////////////////////////////////
void DARTHinge2Joint::SetVelocity(int /*_index*/, double /*_angle*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
double DARTHinge2Joint::GetMaxForce(int /*_index*/)
{
  gzwarn << "Not implemented!\n";

  return 0;
}


//////////////////////////////////////////////////
void DARTHinge2Joint::SetMaxForce(int /*_index*/, double /*_t*/)
{
  gzwarn << "Not implemented!\n";
}


//////////////////////////////////////////////////
void DARTHinge2Joint::SetForce(int /*_index*/, double /*_torque*/)
{
  gzwarn << "Not implemented!\n";
}
