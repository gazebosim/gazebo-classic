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
#include <string>

#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"

#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/ode/ODEUniversalJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
ODEUniversalJoint::ODEUniversalJoint(dWorldID _worldId, BasePtr _parent)
    : UniversalJoint<ODEJoint>(_parent)
{
  this->jointId = dJointCreateUniversal(_worldId, NULL);
}

//////////////////////////////////////////////////
ODEUniversalJoint::~ODEUniversalJoint()
{
  if (this->applyDamping)
    physics::Joint::DisconnectJointUpdate(this->applyDamping);
}

//////////////////////////////////////////////////
math::Vector3 ODEUniversalJoint::GetAnchor(unsigned int /*index*/) const
{
  dVector3 result;
  if (this->jointId)
    dJointGetUniversalAnchor(this->jointId, result);
  else
    gzerr << "ODE Joint ID is invalid\n";

  return math::Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
void ODEUniversalJoint::SetAnchor(unsigned int /*index*/,
    const math::Vector3 &_anchor)
{
  if (this->childLink) this->childLink->SetEnabled(true);
  if (this->parentLink) this->parentLink->SetEnabled(true);

  if (this->jointId)
    dJointSetUniversalAnchor(this->jointId, _anchor.x, _anchor.y, _anchor.z);
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
math::Vector3 ODEUniversalJoint::GetGlobalAxis(unsigned int _index) const
{
  dVector3 result;

  if (this->jointId)
  {
    // flipping axis 1 and 2 around
    if (_index == UniversalJoint::AXIS_CHILD)
      dJointGetUniversalAxis1(this->jointId, result);
    else if (_index == UniversalJoint::AXIS_PARENT)
      dJointGetUniversalAxis2(this->jointId, result);
    else
      gzerr << "Joint index out of bounds.\n";
  }
  else
    gzerr << "ODE Joint ID is invalid\n";

  return math::Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
void ODEUniversalJoint::SetAxis(unsigned int _index, const math::Vector3 &_axis)
{
  if (this->childLink)
    this->childLink->SetEnabled(true);
  if (this->parentLink)
    this->parentLink->SetEnabled(true);

  /// ODE needs global axis
  math::Quaternion axisFrame = this->GetAxisFrame(_index);
  math::Vector3 globalAxis = axisFrame.RotateVector(_axis);

  if (this->jointId)
  {
    // flipping axis 1 and 2 around
    if (_index == UniversalJoint::AXIS_CHILD)
      dJointSetUniversalAxis1(this->jointId,
        globalAxis.x, globalAxis.y, globalAxis.z);
    else if (_index == UniversalJoint::AXIS_PARENT)
      dJointSetUniversalAxis2(this->jointId,
        globalAxis.x, globalAxis.y, globalAxis.z);
    else
      gzerr << "Joint index out of bounds.\n";
  }
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
math::Angle ODEUniversalJoint::GetAngleImpl(unsigned int _index) const
{
  math::Angle result;

  if (this->jointId)
  {
    // flipping axis 1 and 2 around
    if (_index == UniversalJoint::AXIS_CHILD)
      result = dJointGetUniversalAngle1(this->jointId);
    else if (_index == UniversalJoint::AXIS_PARENT)
      result = dJointGetUniversalAngle2(this->jointId);
    else
      gzerr << "Joint index out of bounds.\n";
  }
  else
    gzerr << "ODE Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
double ODEUniversalJoint::GetVelocity(unsigned int _index) const
{
  double result = 0;

  if (this->jointId)
  {
    // flipping axis 1 and 2 around
    if (_index == UniversalJoint::AXIS_CHILD)
      result = dJointGetUniversalAngle1Rate(this->jointId);
    else if (_index == UniversalJoint::AXIS_PARENT)
      result = dJointGetUniversalAngle2Rate(this->jointId);
    else
      gzerr << "Joint index out of bounds.\n";
  }
  else
    gzerr << "ODE Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
void ODEUniversalJoint::SetVelocity(unsigned int _index, double _angle)
{
  // flipping axis 1 and 2 around
  if (_index == UniversalJoint::AXIS_CHILD)
    this->SetParam(dParamVel, _angle);
  else if (_index == UniversalJoint::AXIS_PARENT)
    this->SetParam(dParamVel2, _angle);
  else
    gzerr << "Joint index out of bounds.\n";
}

//////////////////////////////////////////////////
void ODEUniversalJoint::SetForceImpl(unsigned int _index, double _effort)
{
  if (this->jointId)
  {
    // flipping axis 1 and 2 around
    if (_index == UniversalJoint::AXIS_CHILD)
      dJointAddUniversalTorques(this->jointId, _effort, 0);
    else if (_index == UniversalJoint::AXIS_PARENT)
      dJointAddUniversalTorques(this->jointId, 0, _effort);
    else
      gzerr << "Joint index out of bounds.\n";
  }
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
void ODEUniversalJoint::SetMaxForce(unsigned int _index, double _t)
{
  // flipping axis 1 and 2 around
  if (_index == UniversalJoint::AXIS_CHILD)
    this->SetParam(dParamFMax, _t);
  else if (_index == UniversalJoint::AXIS_PARENT)
    this->SetParam(dParamFMax2, _t);
  else
    gzerr << "Joint index out of bounds.\n";
}

//////////////////////////////////////////////////
double ODEUniversalJoint::GetMaxForce(unsigned int _index)
{
  // flipping axis 1 and 2 around
  if (_index == UniversalJoint::AXIS_CHILD)
    return ODEJoint::GetParam(dParamFMax);
  else if (_index == UniversalJoint::AXIS_PARENT)
    return ODEJoint::GetParam(dParamFMax2);

  gzerr << "Joint index out of bounds.\n";
  return 0;
}

//////////////////////////////////////////////////
void ODEUniversalJoint::SetParam(unsigned int _parameter, double _value)
{
  ODEJoint::SetParam(_parameter, _value);

  if (this->jointId)
    dJointSetUniversalParam(this->jointId, _parameter, _value);
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
bool ODEUniversalJoint::SetHighStop(
  unsigned int _index, const math::Angle &_angle)
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
bool ODEUniversalJoint::SetLowStop(
  unsigned int _index, const math::Angle &_angle)
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
bool ODEUniversalJoint::SetParam(
  const std::string &_key, unsigned int _index, const boost::any &_value)
{
  if (_key == "stop_erp")
  {
    try
    {
      switch (_index)
      {
        case UniversalJoint::AXIS_CHILD:
          this->SetParam(dParamStopERP, boost::any_cast<double>(_value));
          break;
        case UniversalJoint::AXIS_PARENT:
          this->SetParam(dParamStopERP2, boost::any_cast<double>(_value));
          break;
        default:
          gzerr << "Invalid index[" << _index << "]\n";
          return false;
      };
    }
    catch(const boost::bad_any_cast &e)
    {
      gzerr << "boost any_cast error:" << e.what() << "\n";
      return false;
    }
  }
  else if (_key == "stop_cfm")
  {
    try
    {
      switch (_index)
      {
        case UniversalJoint::AXIS_CHILD:
          this->SetParam(dParamStopCFM, boost::any_cast<double>(_value));
          break;
        case UniversalJoint::AXIS_PARENT:
          this->SetParam(dParamStopCFM2, boost::any_cast<double>(_value));
          break;
        default:
          gzerr << "Invalid index[" << _index << "]\n";
          return false;
      };
    }
    catch(const boost::bad_any_cast &e)
    {
      gzerr << "boost any_cast error:" << e.what() << "\n";
      return false;
    }
  }
  else if (_key == "hi_stop")
  {
    try
    {
      switch (_index)
      {
        case UniversalJoint::AXIS_CHILD:
          this->SetParam(dParamHiStop, boost::any_cast<double>(_value));
          break;
        case UniversalJoint::AXIS_PARENT:
          this->SetParam(dParamHiStop2, boost::any_cast<double>(_value));
          break;
        default:
          gzerr << "Invalid index[" << _index << "]\n";
          return false;
      };
    }
    catch(const boost::bad_any_cast &e)
    {
      gzerr << "boost any_cast error:" << e.what() << "\n";
      return false;
    }
  }
  else if (_key == "lo_stop")
  {
    try
    {
      switch (_index)
      {
        case UniversalJoint::AXIS_CHILD:
          this->SetParam(dParamLoStop, boost::any_cast<double>(_value));
          break;
        case UniversalJoint::AXIS_PARENT:
          this->SetParam(dParamLoStop2, boost::any_cast<double>(_value));
          break;
        default:
          gzerr << "Invalid index[" << _index << "]\n";
          return false;
      };
    }
    catch(const boost::bad_any_cast &e)
    {
      gzerr << "boost any_cast error:" << e.what() << "\n";
      return false;
    }
  }
  else
  {
    // Overload because we switched axis orders
    return ODEJoint::SetParam(_key, _index, _value);
  }
  return true;
}

//////////////////////////////////////////////////
double ODEUniversalJoint::GetParam(
  const std::string &_key, unsigned int _index)
{
  // Overload because we switched axis orders
  if (_key == "hi_stop")
  {
    try
    {
      switch (_index)
      {
        case UniversalJoint::AXIS_CHILD:
          return ODEJoint::GetParam(dParamHiStop);
        case UniversalJoint::AXIS_PARENT:
          return ODEJoint::GetParam(dParamHiStop2);
        default:
          gzerr << "Invalid index[" << _index << "]\n";
          return 0;
          break;
      };
    }
    catch(const common::Exception &e)
    {
      gzerr << "GetParam error:" << e.GetErrorStr() << "\n";
      return 0;
    }
  }
  else if (_key == "lo_stop")
  {
    try
    {
      switch (_index)
      {
        case UniversalJoint::AXIS_CHILD:
          return ODEJoint::GetParam(dParamLoStop);
        case UniversalJoint::AXIS_PARENT:
          return ODEJoint::GetParam(dParamLoStop2);
        default:
          gzerr << "Invalid index[" << _index << "]\n";
          return 0;
          break;
      };
    }
    catch(const common::Exception &e)
    {
      gzerr << "GetParam error:" << e.GetErrorStr() << "\n";
      return 0;
    }
  }
  else
  {
    return ODEJoint::GetParam(_key, _index);
  }
}
