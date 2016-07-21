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
#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"

#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/ode/ODEJointPrivate.hh"
#include "gazebo/physics/ode/ODEUniversalJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
ODEUniversalJoint::ODEUniversalJoint(dWorldID _worldId, BasePtr _parent)
: UniversalJoint<ODEJoint>(_parent)
{
  this->odeJointDPtr->jointId = dJointCreateUniversal(_worldId, nullptr);
}

//////////////////////////////////////////////////
ODEUniversalJoint::~ODEUniversalJoint()
{
  this->odeJointDPtr->applyDamping.reset();
}

//////////////////////////////////////////////////
ignition::math::Vector3d ODEUniversalJoint::Anchor(
    const unsigned int /*index*/) const
{
  dVector3 result;
  if (this->odeJointDPtr->jointId)
    dJointGetUniversalAnchor(this->odeJointDPtr->jointId, result);
  else
  {
    gzerr << "ODE Joint ID is invalid\n";
    return ignition::math::Vector3d::Zero;
  }

  return ignition::math::Vector3d(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
void ODEUniversalJoint::SetAnchor(const unsigned int /*index*/,
    const ignition::math::Vector3d &_anchor)
{
  if (this->odeJointDPtr->childLink)
    this->odeJointDPtr->childLink->SetEnabled(true);
  if (this->odeJointDPtr->parentLink)
    this->odeJointDPtr->parentLink->SetEnabled(true);

  if (this->odeJointDPtr->jointId)
  {
    dJointSetUniversalAnchor(this->odeJointDPtr->jointId,
        _anchor.X(), _anchor.Y(), _anchor.Z());
  }
  else
  {
    gzerr << "ODE Joint ID is invalid\n";
  }
}

//////////////////////////////////////////////////
ignition::math::Vector3d ODEUniversalJoint::GlobalAxis(
    const unsigned int _index) const
{
  dVector3 result;

  if (this->odeJointDPtr->jointId)
  {
    // flipping axis 1 and 2 around
    if (_index == UniversalJoint::AXIS_CHILD)
      dJointGetUniversalAxis1(this->odeJointDPtr->jointId, result);
    else if (_index == UniversalJoint::AXIS_PARENT)
      dJointGetUniversalAxis2(this->odeJointDPtr->jointId, result);
    else
    {
      gzerr << "Joint index out of bounds.\n";
      return ignition::math::Vector3d::Zero;
    }
  }
  else
  {
    gzerr << "ODE Joint ID is invalid\n";
    return ignition::math::Vector3d::Zero;
  }

  return ignition::math::Vector3d(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
void ODEUniversalJoint::SetAxis(const unsigned int _index,
    const ignition::math::Vector3d &_axis)
{
  if (this->odeJointDPtr->childLink)
    this->odeJointDPtr->childLink->SetEnabled(true);
  if (this->odeJointDPtr->parentLink)
    this->odeJointDPtr->parentLink->SetEnabled(true);

  /// ODE needs global axis
  ignition::math::Quaterniond axisFrame = this->AxisFrame(_index);
  ignition::math::Vector3d globalAxis = axisFrame.RotateVector(_axis);

  if (this->odeJointDPtr->jointId)
  {
    // flipping axis 1 and 2 around
    if (_index == UniversalJoint::AXIS_CHILD)
    {
      dJointSetUniversalAxis1(this->odeJointDPtr->jointId,
        globalAxis.X(), globalAxis.Y(), globalAxis.Z());
    }
    else if (_index == UniversalJoint::AXIS_PARENT)
    {
      dJointSetUniversalAxis2(this->odeJointDPtr->jointId,
        globalAxis.X(), globalAxis.Y(), globalAxis.Z());
    }
    else
    {
      gzerr << "Joint index out of bounds.\n";
    }
  }
  else
  {
    gzerr << "ODE Joint ID is invalid\n";
  }
}

//////////////////////////////////////////////////
ignition::math::Angle ODEUniversalJoint::AngleImpl(
    const unsigned int _index) const
{
  ignition::math::Angle result;

  if (this->odeJointDPtr->jointId)
  {
    // flipping axis 1 and 2 around
    if (_index == UniversalJoint::AXIS_CHILD)
      result = dJointGetUniversalAngle1(this->odeJointDPtr->jointId);
    else if (_index == UniversalJoint::AXIS_PARENT)
      result = dJointGetUniversalAngle2(this->odeJointDPtr->jointId);
    else
      gzerr << "Joint index out of bounds.\n";
  }
  else
    gzerr << "ODE Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
double ODEUniversalJoint::Velocity(const unsigned int _index) const
{
  double result = 0;

  if (this->odeJointDPtr->jointId)
  {
    // flipping axis 1 and 2 around
    if (_index == UniversalJoint::AXIS_CHILD)
      result = dJointGetUniversalAngle1Rate(this->odeJointDPtr->jointId);
    else if (_index == UniversalJoint::AXIS_PARENT)
      result = dJointGetUniversalAngle2Rate(this->odeJointDPtr->jointId);
    else
      gzerr << "Joint index out of bounds.\n";
  }
  else
    gzerr << "ODE Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
void ODEUniversalJoint::SetVelocity(const unsigned int _index,
    const double _angle)
{
  this->SetVelocityMaximal(_index, _angle);
}

//////////////////////////////////////////////////
void ODEUniversalJoint::SetForceImpl(const unsigned int _index,
    const double _effort)
{
  if (this->odeJointDPtr->jointId)
  {
    // flipping axis 1 and 2 around
    if (_index == UniversalJoint::AXIS_CHILD)
      dJointAddUniversalTorques(this->odeJointDPtr->jointId, _effort, 0);
    else if (_index == UniversalJoint::AXIS_PARENT)
      dJointAddUniversalTorques(this->odeJointDPtr->jointId, 0, _effort);
    else
      gzerr << "Joint index out of bounds.\n";
  }
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
double ODEUniversalJoint::Param(const unsigned int _parameter) const
{
  double result = 0;

  if (this->odeJointDPtr->jointId)
    result = dJointGetUniversalParam(this->odeJointDPtr->jointId, _parameter);
  else
    gzerr << "ODE Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
void ODEUniversalJoint::SetParam(const unsigned int _parameter,
    const double _value)
{
  ODEJoint::SetParam(_parameter, _value);

  if (this->odeJointDPtr->jointId)
    dJointSetUniversalParam(this->odeJointDPtr->jointId, _parameter, _value);
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
bool ODEUniversalJoint::SetHighStop(const unsigned int _index,
    const ignition::math::Angle &_angle)
{
  // Overload because we switched axis orders
  Joint::SetHighStop(_index, _angle);
  switch (_index)
  {
    case UniversalJoint::AXIS_CHILD:
      this->SetParam(dParamHiStop, _angle.Radian());
      return true;
    case UniversalJoint::AXIS_PARENT:
      this->SetParam(dParamHiStop2, _angle.Radian());
      return true;
    default:
      gzerr << "Invalid index[" << _index << "]\n";
      return false;
  };
}

//////////////////////////////////////////////////
bool ODEUniversalJoint::SetLowStop(const unsigned int _index,
    const ignition::math::Angle &_angle)
{
  // Overload because we switched axis orders
  Joint::SetLowStop(_index, _angle);
  switch (_index)
  {
    case UniversalJoint::AXIS_CHILD:
      this->SetParam(dParamLoStop, _angle.Radian());
      return true;
    case UniversalJoint::AXIS_PARENT:
      this->SetParam(dParamLoStop2, _angle.Radian());
      return true;
    default:
      gzerr << "Invalid index[" << _index << "]\n";
      return false;
  };
}

//////////////////////////////////////////////////
bool ODEUniversalJoint::SetParam(const std::string &_key,
    const unsigned int _index, const boost::any &_value)
{
  // Axis parameters for multi-axis joints use a group bitmask
  // to identify the variable.
  unsigned int group;
  switch (_index)
  {
    case UniversalJoint::AXIS_CHILD:
      group = dParamGroup1;
      break;
    case UniversalJoint::AXIS_PARENT:
      group = dParamGroup2;
      break;
    default:
      gzerr << "Invalid index[" << _index << "]\n";
      return false;
  };

  try
  {
    if (_key == "stop_erp")
    {
      this->SetParam(dParamStopERP | group, boost::any_cast<double>(_value));
    }
    else if (_key == "stop_cfm")
    {
      this->SetParam(dParamStopCFM | group, boost::any_cast<double>(_value));
    }
    else if (_key == "friction")
    {
      this->SetParam(dParamVel | group, 0.0);
      this->SetParam(dParamFMax | group, boost::any_cast<double>(_value));
    }
    else if (_key == "hi_stop")
    {
      this->SetParam(dParamHiStop | group, boost::any_cast<double>(_value));
    }
    else if (_key == "lo_stop")
    {
      this->SetParam(dParamLoStop | group, boost::any_cast<double>(_value));
    }
    else
    {
      // Overload because we switched axis orders
      return ODEJoint::SetParam(_key, _index, _value);
    }
  }
  catch(const boost::bad_any_cast &e)
  {
    gzerr << "boost any_cast error during "
          << "SetParam('" << _key << "'): "
          << e.what()
          << std::endl;
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
double ODEUniversalJoint::Param(const std::string &_key,
    const unsigned int _index) const
{
  // Axis parameters for multi-axis joints use a group bitmask
  // to identify the variable.
  unsigned int group;
  switch (_index)
  {
    case UniversalJoint::AXIS_CHILD:
      group = dParamGroup1;
      break;
    case UniversalJoint::AXIS_PARENT:
      group = dParamGroup2;
      break;
    default:
      gzerr << "Invalid index[" << _index << "]\n";
      return false;
  };

  // Overload because we switched axis orders
  try
  {
    if (_key == "friction")
    {
      return this->Param(dParamFMax | group);
    }
    else if (_key == "hi_stop")
    {
      return this->HighStop(_index).Radian();
    }
    else if (_key == "lo_stop")
    {
      return this->LowStop(_index).Radian();
    }
    else
    {
      return ODEJoint::Param(_key, _index);
    }
  }
  catch(const common::Exception &e)
  {
    gzerr << "Error during "
          << "Param('" << _key << "'): "
          << e.GetErrorStr()
          << std::endl;
    return 0;
  }
}
