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
PluginScrewJoint::PluginScrewJoint(dWorldID _worldId, BasePtr _parent)
    : ScrewJoint<PluginJoint>(_parent)
{
  this->jointId = dJointCreateScrew(_worldId, NULL);
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
  dVector3 result;
  // initialize to 0
  result[0] = result[1] = result[2] = 0.0;

  if (this->jointId)
    dJointGetScrewAnchor(this->jointId, result);
  else
    gzerr << "Plugin Joint ID is invalid, returning 0 vector.\n";

  return math::Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
void PluginScrewJoint::SetAnchor(unsigned int /*index*/,
    const math::Vector3 &_anchor)
{
  if (!this->jointId)
  {
    gzerr << "Plugin Joint ID is invalid, anchor not set.\n";
    return;
  }

  if (this->childLink)
    this->childLink->SetEnabled(true);
  if (this->parentLink)
    this->parentLink->SetEnabled(true);

  if (this->jointId)
    dJointSetScrewAnchor(this->jointId, _anchor.x, _anchor.y, _anchor.z);
}

//////////////////////////////////////////////////
math::Vector3 PluginScrewJoint::GetGlobalAxis(unsigned int /*_index*/) const
{
  dVector3 result;

  if (this->jointId)
    dJointGetScrewAxis(this->jointId, result);
  else
    gzerr << "Plugin Joint ID is invalid\n";

  return math::Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
void PluginScrewJoint::SetAxis(unsigned int /*_index*/, const math::Vector3 &_axis)
{
  if (this->childLink)
    this->childLink->SetEnabled(true);
  if (this->parentLink)
    this->parentLink->SetEnabled(true);

  /// Plugin needs global axis
  /// \TODO: currently we assume joint axis is specified in model frame,
  /// this is incorrect, and should be corrected to be
  /// joint frame which is specified in child link frame.
  math::Vector3 globalAxis = _axis;
  if (this->parentLink)
    globalAxis =
      this->GetParent()->GetModel()->GetWorldPose().rot.RotateVector(_axis);

  if (this->jointId)
    dJointSetScrewAxis(this->jointId, globalAxis.x, globalAxis.y, globalAxis.z);
  else
    gzerr << "Plugin Joint ID is invalid\n";
}

//////////////////////////////////////////////////
math::Angle PluginScrewJoint::GetAngleImpl(unsigned int _index) const
{
  math::Angle result;
  if (this->jointId)
  {
    if (_index < this->GetAngleCount())
    {
      if (_index == 0)
        result = dJointGetScrewAngle(this->jointId);
      else if (_index == 1)
        result = dJointGetScrewPosition(this->jointId);
    }
    else
    {
      gzwarn << "PluginScrewJoint::GetAngleImpl(" << _index
             << "): invalid index exceeds allowed range("
             << this->GetAngleCount() << ").\n";
    }
  }
  else
    gzerr << "Plugin Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
double PluginScrewJoint::GetVelocity(unsigned int _index) const
{
  double result = 0;

  if (this->jointId)
  {
    if (_index < this->GetAngleCount())
    {
      if (_index == 0)
        result = dJointGetScrewAngleRate(this->jointId);
      else if (_index == 1)
        result = dJointGetScrewPositionRate(this->jointId);
    }
    else
    {
      gzwarn << "PluginScrewJoint::GetAngleImpl(" << _index
             << "): invalid index exceeds allowed range("
             << this->GetAngleCount() << ").\n";
    }
  }
  else
    gzerr << "Plugin Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
void PluginScrewJoint::SetVelocity(unsigned int /*index*/, double _angle)
{
  this->SetParam(dParamVel, _angle);
}

//////////////////////////////////////////////////
void PluginScrewJoint::SetThreadPitch(unsigned int /*_index*/, double _threadPitch)
{
  if (this->jointId)
  {
    /// \TODO: create an issue on making thread pitch = translation / angle
    /// \TODO: create an issue on making thread pitch = translation / angle
    dJointSetScrewThreadPitch(this->jointId, -_threadPitch);
  }
  else
    gzerr << "Plugin Joint ID is invalid\n";
}

//////////////////////////////////////////////////
void PluginScrewJoint::SetThreadPitch(double _threadPitch)
{
  if (this->jointId)
  {
    /// \TODO: create an issue on making thread pitch = translation / angle
    dJointSetScrewThreadPitch(this->jointId, -_threadPitch);
  }
  else
    gzerr << "Plugin Joint ID is invalid\n";
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
  if (this->jointId)
  {
    // dJointAddScrewForce(this->jointId, _effort);
    dJointAddScrewTorque(this->jointId, _effort);
  }
  else
    gzerr << "Plugin Joint ID is invalid\n";
}

//////////////////////////////////////////////////
void PluginScrewJoint::SetParam(unsigned int _parameter, double _value)
{
  PluginJoint::SetParam(_parameter, _value);

  if (this->jointId)
    dJointSetScrewParam(this->jointId, _parameter, _value);
  else
    gzerr << "Plugin Joint ID is invalid\n";
}

//////////////////////////////////////////////////
double PluginScrewJoint::GetParam(unsigned int _parameter) const
{
  double result = 0;

  if (this->jointId)
    result = dJointGetScrewParam(this->jointId, _parameter);
  else
    gzerr << "Plugin Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
void PluginScrewJoint::SetMaxForce(unsigned int /*_index*/, double _t)
{
  this->SetParam(dParamFMax, _t);
}

//////////////////////////////////////////////////
double PluginScrewJoint::GetMaxForce(unsigned int /*_index*/)
{
  return this->GetParam(dParamFMax);
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
