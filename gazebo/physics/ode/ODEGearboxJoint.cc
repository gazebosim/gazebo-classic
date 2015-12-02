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
#include "gazebo/physics/ode/ODELink.hh"
#include "gazebo/physics/ode/ODEGearboxJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
ODEGearboxJoint::ODEGearboxJoint(dWorldID _worldId, BasePtr _parent)
    : GearboxJoint<ODEJoint>(_parent)
{
  this->jointId = dJointCreateGearbox(_worldId, NULL);
}

//////////////////////////////////////////////////
ODEGearboxJoint::~ODEGearboxJoint()
{
}

//////////////////////////////////////////////////
void ODEGearboxJoint::Init()
{
  Joint::Init();
  if (!this->referenceBody.empty())
  {
    LinkPtr link = this->model->GetLink(this->referenceBody);
    this->SetReferenceBody(link);
  }
}

//////////////////////////////////////////////////
void ODEGearboxJoint::Load(sdf::ElementPtr _sdf)
{
  GearboxJoint<ODEJoint>::Load(_sdf);

  this->SetGearboxRatio(this->gearRatio);
}

//////////////////////////////////////////////////
void ODEGearboxJoint::SetReferenceBody(LinkPtr _body)
{
  ODELinkPtr odelink = boost::dynamic_pointer_cast<ODELink>(_body);
  dBodyID refId;
  if (odelink == NULL)
  {
    gzwarn << "Reference body not valid, using inertial frame.\n";
    refId = 0;
  }
  else
  {
    refId = odelink->GetODEId();
  }

  dJointSetGearboxReferenceBody(this->jointId, refId);
}

//////////////////////////////////////////////////
void ODEGearboxJoint::SetReferenceBodyParent(LinkPtr _body)
{
  ODELinkPtr odelink = boost::dynamic_pointer_cast<ODELink>(_body);
  dBodyID refId;
  if (odelink == NULL)
  {
    gzwarn << "Reference body not valid, using inertial frame.\n";
    refId = 0;
  }
  else
  {
    refId = odelink->GetODEId();
  }

  dJointSetGearboxReferenceBody1(this->jointId, refId);
}

//////////////////////////////////////////////////
void ODEGearboxJoint::SetReferenceBodyChild(LinkPtr _body)
{
  ODELinkPtr odelink = boost::dynamic_pointer_cast<ODELink>(_body);
  dBodyID refId;
  if (odelink == NULL)
  {
    gzwarn << "Reference body not valid, using inertial frame.\n";
    refId = 0;
  }
  else
  {
    refId = odelink->GetODEId();
  }

  dJointSetGearboxReferenceBody2(this->jointId, refId);
}

//////////////////////////////////////////////////
void ODEGearboxJoint::SetGearboxRatio(double _gearRatio)
{
  this->gearRatio = _gearRatio;
  dJointSetGearboxRatio(this->jointId, _gearRatio);
}

//////////////////////////////////////////////////
math::Vector3 ODEGearboxJoint::GetGlobalAxis(unsigned int _index) const
{
  dVector3 result;

  if (_index == 0)
    dJointGetGearboxAxis1(this->jointId, result);
  else if (_index == 1)
    dJointGetGearboxAxis2(this->jointId, result);
  else
    gzerr << "index [" << _index << "] out of range\n";

  return math::Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
void ODEGearboxJoint::SetAxis(unsigned int _index, const math::Vector3 &_axis)
{
  // ODEJoint::SetAxis(_index, _axis);

  if (this->childLink)
    this->childLink->SetEnabled(true);
  if (this->parentLink)
    this->parentLink->SetEnabled(true);

  /// ODE needs global axis
  math::Quaternion axisFrame = this->GetAxisFrame(_index);
  math::Vector3 globalAxis = axisFrame.RotateVector(_axis);

  if (_index == 0)
  {
    // check if a joint exists between parent link and referenceBody
    // if so, use it to override globalAxis.
    /// \TODO:  To release, we need to do something to keep
    /// backwards compatible.
    bool found = false;
    LinkPtr link = this->model->GetLink(this->referenceBody);
    // check parent link against reference body's parent joints
    if (link)
    {
      for (const auto &j : link->GetParentJoints())
      {
        // GetParentJoints returns joints that connects to
        // this joint's parent link for a normal joint,
        // but gearbox is not really a normal joint, so check
        // both parent and child link of the parent joint
        if (j->GetParent() && j->GetChild())
          gzerr << j->GetParent()->GetName()
                << " : " << j->GetChild()->GetName()
                << " : " << this->parentLink->GetName() << "\n";
        if (j->GetParent() &&
            j->GetParent().get() == this->parentLink.get())
        {
          globalAxis = -j->GetGlobalAxis(_index);
          found = true;
          gzerr << globalAxis << "\n";
          break;
        }
        else if (j->GetChild() &&
                 j->GetChild().get() == this->parentLink.get())
        {
          globalAxis = j->GetGlobalAxis(_index);
          found = true;
          gzerr << globalAxis << "\n";
          break;
        }
      }
    }
    // check parent link against reference body's parent joints
    if (!found && link)
    {
      for (const auto &j : link->GetChildJoints())
      {
        if (j->GetChild())
          gzerr << j->GetChild()->GetName()
                << " : " << this->parentLink->GetName() << "\n";
        if (j->GetParent() &&
            j->GetParent().get() == this->parentLink.get())
        {
          globalAxis = j->GetGlobalAxis(_index);
          found = true;
          gzerr << globalAxis << "\n";
          break;
        }
        else if (j->GetChild() &&
                 j->GetChild().get() == this->parentLink.get())
        {
          globalAxis = j->GetGlobalAxis(_index);
          found = true;
          gzerr << globalAxis << "\n";
          break;
        }
      }
    }

/*
    // check if a joint exists between parent link and referenceBodyParent
    link = this->model->GetLink(this->referenceBodyParent);
    if (!found && link)
    {
      for (const auto &j : link->GetParentJoints())
      {
        if (j->GetParent() &&
            j->GetParent().get() == this->parentLink.get())
        {
          globalAxis = j->GetGlobalAxis(_index);
          found = true;
          gzerr << globalAxis << "\n";
          break;
        }
        else if (j->GetChild() &&
                 j->GetChild().get() == this->parentLink.get())
        {
          globalAxis = j->GetGlobalAxis(_index);
          found = true;
          gzerr << globalAxis << "\n";
          break;
        }
      }
    }
    // check if a joint exists between parent link and referenceBodyParent
    if (!found && link)
    {
      for (const auto &j : link->GetChildJoints())
      {
        if (j->GetParent() &&
            j->GetParent().get() == this->parentLink.get())
        {
          globalAxis = j->GetGlobalAxis(_index);
          found = true;
          gzerr << globalAxis << "\n";
          break;
        }
        else if (j->GetChild() &&
                 j->GetChild().get() == this->parentLink.get())
        {
          globalAxis = j->GetGlobalAxis(_index);
          found = true;
          gzerr << globalAxis << "\n";
          break;
        }
      }
    }
*/
    if (!found)
      gzerr << "not found\n";

    // if not found, default to use user specified axis
    dJointSetGearboxAxis1(this->jointId, globalAxis.x, globalAxis.y,
      globalAxis.z);
  }
  else if (_index == 1)
  {
    // check if a joint exists between child link and referenceBody
    // if so, use it to override globalAxis.
    /// \TODO:  To release, we need to do something to keep
    /// backwards compatible.
    bool found = false;
    LinkPtr link = this->model->GetLink(this->referenceBody);
    // check child link against reference body's parent joints
    if (link)
    {
      for (const auto &j : link->GetParentJoints())
      {
        if (j->GetParent() && j->GetParent().get() == this->childLink.get())
        {
          globalAxis = j->GetGlobalAxis(_index);
          found = true;
          gzerr << globalAxis << "\n";
          break;
        }
        else if (j->GetChild() && j->GetChild().get() == this->childLink.get())
        {
          globalAxis = j->GetGlobalAxis(_index);
          found = true;
          gzerr << globalAxis << "\n";
          break;
        }
      }
    }
    // check child link against reference body's child joints
    if (!found && link)
    {
      for (const auto &j : link->GetChildJoints())
      {
        if (j->GetParent() && j->GetParent().get() == this->childLink.get())
        {
          globalAxis = j->GetGlobalAxis(_index);
          found = true;
          gzerr << globalAxis << "\n";
          break;
        }
        else if (j->GetChild() && j->GetChild().get() == this->childLink.get())
        {
          globalAxis = j->GetGlobalAxis(_index);
          found = true;
          gzerr << globalAxis << "\n";
          break;
        }
      }
    }

/*
    // check if a joint exists between parent link and referenceBodyChild
    link = this->model->GetLink(this->referenceBodyChild);
    if (!found && link)
    {
      for (const auto &j : link->GetParentJoints())
      {
        if (j->GetParent() &&
            j->GetParent().get() == this->childLink.get())
        {
          globalAxis = j->GetGlobalAxis(_index);
          found = true;
          gzerr << globalAxis << "\n";
          break;
        }
        else if (j->GetChild() &&
                 j->GetChild().get() == this->childLink.get())
        {
          globalAxis = j->GetGlobalAxis(_index);
          found = true;
          gzerr << globalAxis << "\n";
          break;
        }
      }
    }
    // check if a joint exists between parent link and referenceBodyParent
    if (!found && link)
    {
      for (const auto &j : link->GetChildJoints())
      {
        if (j->GetParent() &&
            j->GetParent().get() == this->childLink.get())
        {
          globalAxis = j->GetGlobalAxis(_index);
          found = true;
          gzerr << globalAxis << "\n";
          break;
        }
        else if (j->GetChild() &&
             j->GetChild().get() == this->childLink.get())
        {
          globalAxis = j->GetGlobalAxis(_index);
          found = true;
          gzerr << globalAxis << "\n";
          break;
        }
      }
    }
*/

    if (!found)
      gzerr << "not found\n";

    // if not found, default to use user specified axis
    dJointSetGearboxAxis2(this->jointId, globalAxis.x, globalAxis.y,
      globalAxis.z);
  }
  else
    gzerr << "index [" << _index << "] out of range\n";

  ODEJoint::SetAxis(_index, axisFrame.RotateVectorReverse(globalAxis));
}

//////////////////////////////////////////////////
math::Angle ODEGearboxJoint::GetAngleImpl(unsigned int /*index*/) const
{
  gzlog << "GetAngle not implemented for gearbox\n";
  return math::Angle(0);
}

//////////////////////////////////////////////////
double ODEGearboxJoint::GetVelocity(unsigned int /*index*/) const
{
  gzlog << "GetVelocity not implemented for gearbox\n";
  return 0;
}

//////////////////////////////////////////////////
void ODEGearboxJoint::SetVelocity(unsigned int /*index*/, double /*_angle*/)
{
  gzlog << "SetVelocity not implemented for gearbox\n";
  return;
}

//////////////////////////////////////////////////
void ODEGearboxJoint::SetForceImpl(unsigned int /*_index*/, double /*_effort*/)
{
  if (this->jointId)
    gzlog << "SetForce not implemented for gearbox\n";
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
double ODEGearboxJoint::GetParam(unsigned int /*_parameter*/) const
{
  gzlog << "GetParam not implemented for gearbox\n";
  return 0;
}

//////////////////////////////////////////////////
void ODEGearboxJoint::SetParam(unsigned int /*_parameter*/, double /*_value*/)
{
  gzlog << "SetParam not implemented for gearbox\n";
  return;
}

//////////////////////////////////////////////////
math::Vector3 ODEGearboxJoint::GetAnchor(unsigned int /*_index*/) const
{
  dVector3 result;
  gzlog << "ODEGearboxJoint::GetAnchor not implemented.\n";
  return math::Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
void ODEGearboxJoint::SetAnchor(unsigned int /*_index*/,
  const math::Vector3 &/*_anchor*/)
{
  gzlog << "ODEGearboxJoint::SetAnchor not implemented.\n";
}
