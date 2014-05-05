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
#include "gazebo/physics/ode/ODEScrewJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
ODEScrewJoint::ODEScrewJoint(dWorldID _worldId, BasePtr _parent)
    : ScrewJoint<ODEJoint>(_parent)
{
  this->jointId = dJointCreateScrew(_worldId, NULL);
}

//////////////////////////////////////////////////
ODEScrewJoint::~ODEScrewJoint()
{
  if (this->applyDamping)
    physics::Joint::DisconnectJointUpdate(this->applyDamping);
}

//////////////////////////////////////////////////
void ODEScrewJoint::Load(sdf::ElementPtr _sdf)
{
  ScrewJoint<ODEJoint>::Load(_sdf);
  this->SetThreadPitch(this->threadPitch);
}

//////////////////////////////////////////////////
math::Vector3 ODEScrewJoint::GetAnchor(unsigned int /*index*/) const
{
  dVector3 result;
  // initialize to 0
  result[0] = result[1] = result[2] = 0.0;

  if (this->jointId)
    dJointGetScrewAnchor(this->jointId, result);
  else
    gzerr << "ODE Joint ID is invalid, returning 0 vector.\n";

  return math::Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
void ODEScrewJoint::SetAnchor(unsigned int /*index*/,
    const math::Vector3 &_anchor)
{
  if (!this->jointId)
  {
    gzerr << "ODE Joint ID is invalid, anchor not set.\n";
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
math::Vector3 ODEScrewJoint::GetGlobalAxis(unsigned int /*_index*/) const
{
  dVector3 result;

  if (this->jointId)
    dJointGetScrewAxis(this->jointId, result);
  else
    gzerr << "ODE Joint ID is invalid\n";

  return math::Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
void ODEScrewJoint::SetAxis(unsigned int /*_index*/, const math::Vector3 &_axis)
{
  if (this->childLink)
    this->childLink->SetEnabled(true);
  if (this->parentLink)
    this->parentLink->SetEnabled(true);

  /// ODE needs global axis
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
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
math::Angle ODEScrewJoint::GetAngleImpl(unsigned int _index) const
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
      gzwarn << "ODEScrewJoint::GetAngleImpl(" << _index
             << "): invalid index exceeds allowed range("
             << this->GetAngleCount() << ").\n";
    }
  }
  else
    gzerr << "ODE Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
double ODEScrewJoint::GetVelocity(unsigned int _index) const
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
      gzwarn << "ODEScrewJoint::GetAngleImpl(" << _index
             << "): invalid index exceeds allowed range("
             << this->GetAngleCount() << ").\n";
    }
  }
  else
    gzerr << "ODE Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
void ODEScrewJoint::SetVelocity(unsigned int /*index*/, double _angle)
{
  this->SetParam(dParamVel, _angle);
}

//////////////////////////////////////////////////
void ODEScrewJoint::SetThreadPitch(unsigned int /*_index*/, double _threadPitch)
{
  if (this->jointId)
  {
    /// \TODO: create an issue on making thread pitch = translation / angle
    /// \TODO: create an issue on making thread pitch = translation / angle
    dJointSetScrewThreadPitch(this->jointId, -_threadPitch);
  }
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
void ODEScrewJoint::SetThreadPitch(double _threadPitch)
{
  if (this->jointId)
  {
    /// \TODO: create an issue on making thread pitch = translation / angle
    dJointSetScrewThreadPitch(this->jointId, -_threadPitch);
  }
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
double ODEScrewJoint::GetThreadPitch(unsigned int /*_index*/)
{
  return this->GetThreadPitch();
}

//////////////////////////////////////////////////
double ODEScrewJoint::GetThreadPitch()
{
  return this->threadPitch;
}

//////////////////////////////////////////////////
void ODEScrewJoint::SetForceImpl(unsigned int /*_index*/, double _effort)
{
  if (this->jointId)
  {
    // dJointAddScrewForce(this->jointId, _effort);
    dJointAddScrewTorque(this->jointId, _effort);
  }
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
void ODEScrewJoint::SetParam(unsigned int _parameter, double _value)
{
  ODEJoint::SetParam(_parameter, _value);

  if (this->jointId)
    dJointSetScrewParam(this->jointId, _parameter, _value);
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
double ODEScrewJoint::GetParam(unsigned int _parameter) const
{
  double result = 0;

  if (this->jointId)
    result = dJointGetScrewParam(this->jointId, _parameter);
  else
    gzerr << "ODE Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
void ODEScrewJoint::SetMaxForce(unsigned int /*_index*/, double _t)
{
  this->SetParam(dParamFMax, _t);
}

//////////////////////////////////////////////////
double ODEScrewJoint::GetMaxForce(unsigned int /*_index*/)
{
  return this->GetParam(dParamFMax);
}

//////////////////////////////////////////////////
void ODEScrewJoint::SetAttribute(const std::string &_key,
  unsigned int _index, const boost::any &_value)
{
  this->SetParam(_key, _index, _value);
}

//////////////////////////////////////////////////
bool ODEScrewJoint::SetParam(const std::string &_key,
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
    return ODEJoint::SetParam(_key, _index, _value);

  return true;
}

//////////////////////////////////////////////////
double ODEScrewJoint::GetParam(const std::string &_key, unsigned int _index)
{
  if (_key  == "thread_pitch")
    return this->threadPitch;
  else
    return ODEJoint::GetParam(_key, _index);
}

//////////////////////////////////////////////////
double ODEScrewJoint::GetAttribute(const std::string &_key, unsigned int _index)
{
  return this->GetParam(_key, _index);
}
