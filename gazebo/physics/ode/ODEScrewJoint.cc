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
#include <string>

#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"

#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/ode/ODEJointPrivate.hh"
#include "gazebo/physics/ode/ODEScrewJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
ODEScrewJoint::ODEScrewJoint(dWorldID _worldId, BasePtr _parent)
: ScrewJoint<ODEJoint>(_parent)
{
  this->odeJointDPtr->jointId = dJointCreateScrew(_worldId, NULL);
}

//////////////////////////////////////////////////
ODEScrewJoint::~ODEScrewJoint()
{
  if (this->odeJointDPtr->applyDamping)
    physics::Joint::DisconnectJointUpdate(this->odeJointDPtr->applyDamping);
}

//////////////////////////////////////////////////
void ODEScrewJoint::Load(sdf::ElementPtr _sdf)
{
  ScrewJoint<ODEJoint>::Load(_sdf);
  this->SetThreadPitch(this->threadPitch);
}

//////////////////////////////////////////////////
ignition::math::Vector3d ODEScrewJoint::Anchor(unsigned int /*index*/) const
{
  dVector3 result;
  // initialize to 0
  result[0] = result[1] = result[2] = 0.0;

  if (this->odeJointDPtr->jointId)
    dJointGetScrewAnchor(this->odeJointDPtr->jointId, result);
  else
  {
    gzerr << "ODE Joint ID is invalid, returning 0 vector.\n";
    return ignition::math::Vector3d::Zero;
  }

  return ignition::math::Vector3d(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
void ODEScrewJoint::SetAnchor(unsigned int /*index*/,
    const ignition::math::Vector3d &_anchor)
{
  if (!this->odeJointDPtr->jointId)
  {
    gzerr << "ODE Joint ID is invalid, anchor not set.\n";
    return;
  }

  if (this->odeJointDPtr->childLink)
    this->odeJointDPtr->childLink->SetEnabled(true);
  if (this->odeJointDPtr->parentLink)
    this->odeJointDPtr->parentLink->SetEnabled(true);

  if (this->odeJointDPtr->jointId)
  {
    dJointSetScrewAnchor(this->odeJointDPtr->jointId,
        _anchor.X(), _anchor.Y(), _anchor.Z());
  }
}

//////////////////////////////////////////////////
ignition::math::Vector3d ODEScrewJoint::GlobalAxis(
    const unsigned int /*_index*/) const
{
  dVector3 result;

  if (this->odeJointDPtr->jointId)
    dJointGetScrewAxis(this->odeJointDPtr->jointId, result);
  else
  {
    gzerr << "ODE Joint ID is invalid\n";
    return ignition::math::Vector3d::Zero;
  }

  return ignition::math::Vector3d(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
void ODEScrewJoint::SetAxis(const unsigned int /*_index*/,
    const ignition::math::Vector3d &_axis)
{
  if (this->odeJointDPtr->childLink)
    this->odeJointDPtr->childLink->SetEnabled(true);
  if (this->odeJointDPtr->parentLink)
    this->odeJointDPtr->parentLink->SetEnabled(true);

  /// ODE needs global axis
  /// \TODO: currently we assume joint axis is specified in model frame,
  /// this is incorrect, and should be corrected to be
  /// joint frame which is specified in child link frame.
  ignition::math::Vector3d globalAxis = _axis;
  if (this->odeJointDPtr->parentLink)
  {
    globalAxis =
      this->Parent()->Model()->WorldPose().Rot().RotateVector(_axis);
  }

  if (this->odeJointDPtr->jointId)
  {
    dJointSetScrewAxis(this->odeJointDPtr->jointId,
        globalAxis.X(), globalAxis.Y(), globalAxis.Z());
  }
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
ignition::math::Angle ODEScrewJoint::AngleImpl(
    const unsigned int _index) const
{
  ignition::math::Angle result;
  if (this->odeJointDPtr->jointId)
  {
    if (_index < this->AngleCount())
    {
      if (_index == 0)
        result = dJointGetScrewAngle(this->odeJointDPtr->jointId);
      else if (_index == 1)
        result = dJointGetScrewPosition(this->odeJointDPtr->jointId);
    }
    else
    {
      gzwarn << "ODEScrewJoint::GetAngleImpl(" << _index
             << "): invalid index exceeds allowed range("
             << this->AngleCount() << ").\n";
    }
  }
  else
    gzerr << "ODE Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
double ODEScrewJoint::Velocity(const unsigned int _index) const
{
  double result = 0;

  if (this->odeJointDPtr->jointId)
  {
    if (_index < this->AngleCount())
    {
      if (_index == 0)
        result = dJointGetScrewAngleRate(this->odeJointDPtr->jointId);
      else if (_index == 1)
        result = dJointGetScrewPositionRate(this->odeJointDPtr->jointId);
    }
    else
    {
      gzwarn << "ODEScrewJoint::GetAngleImpl(" << _index
             << "): invalid index exceeds allowed range("
             << this->AngleCount() << ").\n";
    }
  }
  else
    gzerr << "ODE Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
void ODEScrewJoint::SetVelocity(const unsigned int /*index*/,
    const double _angle)
{
  this->SetParam(dParamVel, _angle);
}

//////////////////////////////////////////////////
void ODEScrewJoint::SetThreadPitch(const unsigned int /*_index*/,
    const double _threadPitch)
{
  if (this->odeJointDPtr->jointId)
  {
    /// \TODO: create an issue on making thread pitch = translation / angle
    /// \TODO: create an issue on making thread pitch = translation / angle
    dJointSetScrewThreadPitch(this->odeJointDPtr->jointId, -_threadPitch);
  }
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
void ODEScrewJoint::SetThreadPitch(const double _threadPitch)
{
  if (this->odeJointDPtr->jointId)
  {
    /// \TODO: create an issue on making thread pitch = translation / angle
    dJointSetScrewThreadPitch(this->odeJointDPtr->jointId, -_threadPitch);
  }
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
double ODEScrewJoint::ThreadPitch(const unsigned int /*_index*/) const
{
  return this->ThreadPitch();
}

//////////////////////////////////////////////////
double ODEScrewJoint::ThreadPitch() const
{
  return this->threadPitch;
}

//////////////////////////////////////////////////
void ODEScrewJoint::SetForceImpl(const unsigned int /*_index*/,
    const double _effort)
{
  if (this->odeJointDPtr->jointId)
  {
    // dJointAddScrewForce(this->odeJointDPtr->jointId, _effort);
    dJointAddScrewTorque(this->odeJointDPtr->jointId, _effort);
  }
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
void ODEScrewJoint::SetParam(const unsigned int _parameter,
    const double _value)
{
  ODEJoint::SetParam(_parameter, _value);

  if (this->odeJointDPtr->jointId)
    dJointSetScrewParam(this->odeJointDPtr->jointId, _parameter, _value);
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
double ODEScrewJoint::Param(const unsigned int _parameter) const
{
  double result = 0;

  if (this->odeJointDPtr->jointId)
    result = dJointGetScrewParam(this->odeJointDPtr->jointId, _parameter);
  else
    gzerr << "ODE Joint ID is invalid\n";

  return result;
}

/////j////////////////////////////////////////////
bool ODEScrewJoint::SetParam(const std::string &_key,
  const unsigned int _index, const boost::any &_value)
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
double ODEScrewJoint::Param(const std::string &_key,
    const unsigned int _index) const
{
  if (_key  == "thread_pitch")
    return this->ThreadPitch();
  else
    return ODEJoint::Param(_key, _index);
}
