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

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Subscriber.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/Joint.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/PhysicsEngine.hh"

#include "gazebo/physics/JointControllerPrivate.hh"
#include "gazebo/physics/JointController.hh"

using namespace gazebo;
using namespace physics;

/////////////////////////////////////////////////
JointController::JointController(ModelPtr _model)
  : dataPtr(new JointControllerPrivate)
{
  this->dataPtr->model = _model;

  if (this->dataPtr->model && this->dataPtr->model->GetWorld())
  {
    this->dataPtr->node = transport::NodePtr(new transport::Node());
    this->dataPtr->node->Init(this->dataPtr->model->GetWorld()->GetName());

    this->dataPtr->jointCmdSub = this->dataPtr->node->Subscribe(
        std::string("~/") + this->dataPtr->model->GetName() + "/joint_cmd",
        &JointController::OnJointCmd, this);
  }
  else
  {
    gzwarn << "Unable to get world name. "
      << "JointController will not receive commands via messages\n";
  }
}

/////////////////////////////////////////////////
JointController::~JointController()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
void JointController::AddJoint(JointPtr _joint)
{
  this->dataPtr->joints[_joint->GetScopedName()] = _joint;
  this->dataPtr->posPids[_joint->GetScopedName()].Init(
      1, 0.1, 0.01, 1, -1, 1000, -1000);
  this->dataPtr->velPids[_joint->GetScopedName()].Init(
      1, 0.1, 0.01, 1, -1, 1000, -1000);
}

/////////////////////////////////////////////////
void JointController::Reset()
{
  // Reset setpoints and feed-forward.
  this->dataPtr->positions.clear();
  this->dataPtr->velocities.clear();
  this->dataPtr->forces.clear();
  // Should the PID's be reset as well?
  
}

/////////////////////////////////////////////////
void JointController::Update()
{
  common::Time currTime = this->dataPtr->model->GetWorld()->GetSimTime();
  common::Time stepTime = currTime - this->dataPtr->prevUpdateTime;
  this->dataPtr->prevUpdateTime = currTime;

  // Skip the update step if SimTime appears to have gone backward.
  // Negative update time wreaks havok on the integrators.
  // This happens when World::ResetTime is called.
  // TODO: fix this when World::ResetTime is improved
  if (stepTime > 0)
  {
    if (!this->dataPtr->forces.empty())
    {
      std::map<std::string, double>::iterator iter;
      for (iter = this->dataPtr->forces.begin();
          iter != this->dataPtr->forces.end(); ++iter)
      {
        this->dataPtr->joints[iter->first]->SetForce(0, iter->second);
      }
    }

    if (!this->dataPtr->positions.empty())
    {
      std::map<std::string, double>::iterator iter;

      for (iter = this->dataPtr->positions.begin();
           iter != this->dataPtr->positions.end(); ++iter)
      {
        double cmd = this->dataPtr->posPids[iter->first].Update(
            this->dataPtr->joints[iter->first]->GetAngle(0).Radian() -
            iter->second, stepTime);
        this->dataPtr->joints[iter->first]->SetForce(0, cmd);
      }
    }

    if (!this->dataPtr->velocities.empty())
    {
      std::map<std::string, double>::iterator iter;

      for (iter = this->dataPtr->velocities.begin();
           iter != this->dataPtr->velocities.end(); ++iter)
      {
        double cmd = this->dataPtr->velPids[iter->first].Update(
            this->dataPtr->joints[iter->first]->GetVelocity(0) - iter->second,
            stepTime);
        this->dataPtr->joints[iter->first]->SetForce(0, cmd);
      }
    }
  }

  /* enable below if we want to set position kinematically
  if (this->dataPtr->positions.size() > 0)
  {
    std::map<std::string, JointPtr>::iterator iter;
    for (iter = this->dataPtr->joints.begin();
         iter != this->dataPtr->joints.end(); ++iter)
    {
      if (this->dataPtr->positions.find(iter->first) ==
          this->dataPtr->positions.end())
      {
        this->dataPtr->positions[iter->first] =
          iter->second->GetAngle(0).Radian();
      }
    }
    this->SetJointPositions(this->dataPtr->positions);
    this->dataPtr->positions.clear();
  }
  */
}

/////////////////////////////////////////////////
void JointController::OnJointCmd(ConstJointCmdPtr &_msg)
{
  std::map<std::string, JointPtr>::iterator iter;
  iter = this->dataPtr->joints.find(_msg->name());
  if (iter != this->dataPtr->joints.end())
  {
    if (_msg->has_reset() && _msg->reset())
    {
      if (this->dataPtr->forces.find(_msg->name()) !=
          this->dataPtr->forces.end())
      {
        this->dataPtr->forces.erase(this->dataPtr->forces.find(_msg->name()));
      }

      if (this->dataPtr->positions.find(_msg->name()) !=
          this->dataPtr->positions.end())
      {
        this->dataPtr->positions.erase(
            this->dataPtr->positions.find(_msg->name()));
      }

      if (this->dataPtr->velocities.find(_msg->name()) !=
          this->dataPtr->velocities.end())
      {
        this->dataPtr->velocities.erase(
            this->dataPtr->velocities.find(_msg->name()));
      }
    }

    if (_msg->has_force())
      this->dataPtr->forces[_msg->name()] = _msg->force();

    if (_msg->has_position())
    {
      if (_msg->position().has_target())
      {
        if (!this->SetPositionTarget(_msg->name(), _msg->position().target()))
        {
          gzerr << "Unable to set position target for joint["
            << _msg->name() << "]. Joint is not found.\n";
        }
      }

      if (_msg->position().has_p_gain())
      {
        this->dataPtr->posPids[_msg->name()].SetPGain(
            _msg->position().p_gain());
      }

      if (_msg->position().has_i_gain())
      {
        this->dataPtr->posPids[_msg->name()].SetIGain(
            _msg->position().i_gain());
      }

      if (_msg->position().has_d_gain())
      {
        this->dataPtr->posPids[_msg->name()].SetDGain(
            _msg->position().d_gain());
      }

      if (_msg->position().has_i_max())
      {
        this->dataPtr->posPids[_msg->name()].SetIMax(_msg->position().i_max());
      }

      if (_msg->position().has_i_min())
      {
        this->dataPtr->posPids[_msg->name()].SetIMin(_msg->position().i_min());
      }

      if (_msg->position().has_limit())
      {
        this->dataPtr->posPids[_msg->name()].SetCmdMax(
            _msg->position().limit());
        this->dataPtr->posPids[_msg->name()].SetCmdMin(
            -_msg->position().limit());
      }
    }

    if (_msg->has_velocity())
    {
      if (_msg->velocity().has_target())
      {
        if (!this->SetVelocityTarget(_msg->name(), _msg->velocity().target()))
        {
          gzerr << "Unable to set velocity target for joint["
            << _msg->name() << "]. Joint is not found.\n";
        }
      }

      if (_msg->velocity().has_p_gain())
      {
        this->dataPtr->velPids[_msg->name()].SetPGain(
            _msg->velocity().p_gain());
      }

      if (_msg->velocity().has_i_gain())
      {
        this->dataPtr->velPids[_msg->name()].SetIGain(
            _msg->velocity().i_gain());
      }

      if (_msg->velocity().has_d_gain())
      {
        this->dataPtr->velPids[_msg->name()].SetDGain(
            _msg->velocity().d_gain());
      }

      if (_msg->velocity().has_i_max())
      {
        this->dataPtr->velPids[_msg->name()].SetIMax(_msg->velocity().i_max());
      }

      if (_msg->velocity().has_i_min())
      {
        this->dataPtr->velPids[_msg->name()].SetIMin(_msg->velocity().i_min());
      }

      if (_msg->velocity().has_limit())
      {
        this->dataPtr->velPids[_msg->name()].SetCmdMax(
            _msg->velocity().limit());
        this->dataPtr->velPids[_msg->name()].SetCmdMin(
            -_msg->velocity().limit());
      }
    }
  }
  else
    gzerr << "Unable to find joint[" << _msg->name() << "]\n";
}

//////////////////////////////////////////////////
void JointController::SetJointPosition(const std::string & _name,
                                       double _position, int _index)
{
  std::map<std::string, JointPtr>::iterator jiter =
    this->dataPtr->joints.find(_name);

  if (jiter != this->dataPtr->joints.end())
    this->SetJointPosition(jiter->second, _position, _index);
  else
    gzwarn << "SetJointPosition [" << _name << "] not found\n";
}

//////////////////////////////////////////////////
void JointController::SetJointPositions(
    const std::map<std::string, double> & _jointPositions)
{
  // go through all joints in this model and update each one
  //   for each joint update, recursively update all children
  std::map<std::string, JointPtr>::iterator iter;
  std::map<std::string, double>::const_iterator jiter;

  for (iter = this->dataPtr->joints.begin();
      iter != this->dataPtr->joints.end(); ++iter)
  {
    // First try name without scope, i.e. joint_name
    jiter = _jointPositions.find(iter->second->GetName());

    if (jiter == _jointPositions.end())
    {
      // Second try name with scope, i.e. model_name::joint_name
      jiter = _jointPositions.find(iter->second->GetScopedName());
      if (jiter == _jointPositions.end())
        continue;
    }

    this->SetJointPosition(iter->second, jiter->second);
  }
}

//////////////////////////////////////////////////
void JointController::SetJointPosition(
  JointPtr _joint, double _position, int _index)
{
  _joint->SetPosition(_index, _position);
}

/////////////////////////////////////////////////
common::Time JointController::GetLastUpdateTime() const
{
  return this->dataPtr->prevUpdateTime;
}

/////////////////////////////////////////////////
std::map<std::string, JointPtr> JointController::GetJoints() const
{
  return this->dataPtr->joints;
}

/////////////////////////////////////////////////
std::map<std::string, common::PID> JointController::GetPositionPIDs() const
{
  return this->dataPtr->posPids;
}

/////////////////////////////////////////////////
std::map<std::string, common::PID> JointController::GetVelocityPIDs() const
{
  return this->dataPtr->velPids;
}

/////////////////////////////////////////////////
std::map<std::string, double> JointController::GetForces() const
{
  return this->dataPtr->forces;
}

/////////////////////////////////////////////////
std::map<std::string, double> JointController::GetPositions() const
{
  return this->dataPtr->positions;
}

/////////////////////////////////////////////////
std::map<std::string, double> JointController::GetVelocities() const
{
  return this->dataPtr->velocities;
}

//////////////////////////////////////////////////
void JointController::SetPositionPID(const std::string &_jointName,
                                     const common::PID &_pid)
{
  std::map<std::string, JointPtr>::iterator iter;
  iter = this->dataPtr->joints.find(_jointName);

  if (iter != this->dataPtr->joints.end())
    this->dataPtr->posPids[_jointName] = _pid;
  else
    gzerr << "Unable to find joint with name[" << _jointName << "]\n";
}

/////////////////////////////////////////////////
bool JointController::SetPositionTarget(const std::string &_jointName,
    double _target)
{
  bool result = false;

  if (this->dataPtr->posPids.find(_jointName) !=
      this->dataPtr->posPids.end())
  {
    this->dataPtr->positions[_jointName] = _target;
    result = true;
  }

  return result;
}

//////////////////////////////////////////////////
void JointController::SetVelocityPID(const std::string &_jointName,
                                     const common::PID &_pid)
{
  std::map<std::string, JointPtr>::iterator iter;
  iter = this->dataPtr->joints.find(_jointName);

  if (iter != this->dataPtr->joints.end())
    this->dataPtr->velPids[_jointName] = _pid;
  else
    gzerr << "Unable to find joint with name[" << _jointName << "]\n";
}

/////////////////////////////////////////////////
bool JointController::SetVelocityTarget(const std::string &_jointName,
    double _target)
{
  bool result = false;

  if (this->dataPtr->velPids.find(_jointName) !=
      this->dataPtr->velPids.end())
  {
    this->dataPtr->velocities[_jointName] = _target;
    result = true;
  }

  return result;
}
