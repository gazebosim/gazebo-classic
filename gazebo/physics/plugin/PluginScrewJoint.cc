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
/* Desc: A screw or primastic joint
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */

#include <boost/bind.hpp>

#include <string>

#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"

#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/plugin/PluginScrewJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
PluginScrewJoint::PluginScrewJoint(BasePtr _parent)
    : ScrewJoint<PluginJoint>(_parent)
{
}

//////////////////////////////////////////////////
PluginScrewJoint::~PluginScrewJoint()
{
  if (this->applyDamping)
    physics::Joint::DisconnectJointUpdate(this->applyDamping);
}

//////////////////////////////////////////////////
void PluginScrewJoint::Load(sdf::ElementPtr _sdf)
{
  ScrewJoint<PluginJoint>::Load(_sdf);
  this->SetThreadPitch(this->threadPitch);
}

//////////////////////////////////////////////////
math::Vector3 PluginScrewJoint::GetAnchor(unsigned int /*index*/) const
{
  gzerr << "not implemented\n";
  return math::Vector3();
}

//////////////////////////////////////////////////
void PluginScrewJoint::SetAnchor(unsigned int /*index*/,
    const math::Vector3 &_anchor)
{
  if (this->childLink)
    this->childLink->SetEnabled(true);
  if (this->parentLink)
    this->parentLink->SetEnabled(true);
  gzerr << "not implemented\n";
}

//////////////////////////////////////////////////
math::Vector3 PluginScrewJoint::GetGlobalAxis(unsigned int /*_index*/) const
{
  gzerr << "not implemented\n";
  return math::Vector3();
}

//////////////////////////////////////////////////
void PluginScrewJoint::SetAxis(unsigned int /*_index*/, const math::Vector3 &_axis)
{
  if (this->childLink)
    this->childLink->SetEnabled(true);
  if (this->parentLink)
    this->parentLink->SetEnabled(true);

  gzerr << "not implemented\n";
}

//////////////////////////////////////////////////
math::Angle PluginScrewJoint::GetAngleImpl(unsigned int _index) const
{
  math::Angle result;
  gzerr << "not implemented\n";
  return result;
}

//////////////////////////////////////////////////
double PluginScrewJoint::GetVelocity(unsigned int _index) const
{
  double result = 0;
  gzerr << "not implemented\n";
  return result;
}

//////////////////////////////////////////////////
void PluginScrewJoint::SetVelocity(unsigned int /*index*/, double _angle)
{
  gzerr << "not implemented\n";
}

//////////////////////////////////////////////////
void PluginScrewJoint::SetThreadPitch(unsigned int /*_index*/, double _threadPitch)
{
  gzerr << "not implemented\n";
}

//////////////////////////////////////////////////
void PluginScrewJoint::SetThreadPitch(double _threadPitch)
{
  gzerr << "not implemented\n";
}

//////////////////////////////////////////////////
double PluginScrewJoint::GetThreadPitch(unsigned int /*_index*/)
{
  return this->GetThreadPitch();
}

//////////////////////////////////////////////////
double PluginScrewJoint::GetThreadPitch()
{
  return this->threadPitch;
}

//////////////////////////////////////////////////
void PluginScrewJoint::SetForceImpl(unsigned int /*_index*/, double _effort)
{
  gzerr << "not implemented\n";
}

//////////////////////////////////////////////////
void PluginScrewJoint::SetParam(unsigned int _parameter, double _value)
{
  PluginJoint::SetParam(_parameter, _value);
  gzerr << "not implemented\n";
}

//////////////////////////////////////////////////
double PluginScrewJoint::GetParam(unsigned int _parameter) const
{
  double result = 0;
  gzerr << "not implemented\n";
  return result;
}

//////////////////////////////////////////////////
void PluginScrewJoint::SetMaxForce(unsigned int /*_index*/, double _t)
{
  gzerr << "not implemented\n";
}

//////////////////////////////////////////////////
double PluginScrewJoint::GetMaxForce(unsigned int /*_index*/)
{
  gzerr << "not implemented\n";
}

//////////////////////////////////////////////////
bool PluginScrewJoint::SetParam(const std::string &_key,
  unsigned int _index, const boost::any &_value)
{
  if (_key  == "thread_pitch")
  {
    try
    {
      this->threadPitch = boost::any_cast<double>(_value);
    }
    catch(const boost::bad_any_cast &e)
    {
      gzerr << "boost any_cast error:" << e.what() << "\n";
      return false;
    }
  }
  else
    return PluginJoint::SetParam(_key, _index, _value);

  return true;
}

//////////////////////////////////////////////////
double PluginScrewJoint::GetParam(const std::string &_key, unsigned int _index)
{
  if (_key  == "thread_pitch")
    return this->threadPitch;
  else
    return PluginJoint::GetParam(_key, _index);
}
