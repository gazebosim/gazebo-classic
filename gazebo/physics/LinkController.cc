/*
 * Copyright (C) 2012-2013 Open Source Robotics Foundation
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

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Subscriber.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/Wrench.hh"
#include "gazebo/physics/LinkController.hh"
#include "gazebo/physics/PhysicsEngine.hh"

using namespace gazebo;
using namespace physics;

/////////////////////////////////////////////////
LinkController::LinkController(ModelPtr _model)
  : model(_model)
{
  // listen to link_cmd gztopic for moving links
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());
  this->linkCmdSub = this->node->Subscribe(std::string("~/") +
      this->model->GetName() + "/link_cmd",
      &LinkController::OnLinkCmd, this);
}

/////////////////////////////////////////////////
void LinkController::AddLink(LinkPtr _link, math::Pose _pose)
{
  this->links[_link->GetScopedName()] = _link;

  // add _link to list of controlled links,
  // initialized internal control variables for each link.

  // setup pid for controlling link position
  std::vector<common::PID> *posPid = &this->posPids[_link->GetScopedName()];
  if (posPid->size() != 3)  // resize one for each axis
    posPid->resize(3);
  for (unsigned int i = 0; i < posPid->size(); ++i)
  {
    common::PID pid(500.0, 0, 10.0, 0, 0, 1e4, -1e4);
    (*posPid)[i] = pid;
  }

  // setup pid for controlling link orientation
  std::vector<common::PID> *rotPid = &this->rotPids[_link->GetScopedName()];
  if (rotPid->size() != 3)  // resize one for each axis
    rotPid->resize(3);
  for (unsigned int i = 0; i < rotPid->size(); ++i)
  {
    common::PID pid(50.0, 0, 1.0, 0, 0, 1e4, -1e4);
    (*rotPid)[i] = pid;
  }

  // initialize target pose as link's current pose
  this->targetPoses[_link->GetScopedName()] = _pose;

  // initialize command wrench for link to zeros
  this->wrenches[_link->GetScopedName()] = physics::Wrench();

  // Reset Time
  this->prevUpdateTime = this->model->GetWorld()->GetSimTime();
}

/////////////////////////////////////////////////
void LinkController::SetLink(LinkPtr _link, math::Pose _pose)
{
  this->Reset();
  this->links[_link->GetScopedName()] = _link;

  // add _link to list of controlled links,
  // initialized internal control variables for each link.

  // setup pid for controlling link position
  std::vector<common::PID> *posPid = &this->posPids[_link->GetScopedName()];
  if (posPid->size() != 3)  // resize one for each axis
    posPid->resize(3);
  for (unsigned int i = 0; i < posPid->size(); ++i)
  {
    common::PID pid(500.0, 0, 10.0, 0, 0, 1e4, -1e4);
    (*posPid)[i] = pid;
  }

  // setup pid for controlling link orientation
  std::vector<common::PID> *rotPid = &this->rotPids[_link->GetScopedName()];
  if (rotPid->size() != 3)  // resize one for each axis
    rotPid->resize(3);
  for (unsigned int i = 0; i < rotPid->size(); ++i)
  {
    common::PID pid(50.0, 0, 1.0, 0, 0, 1e4, -1e4);
    (*rotPid)[i] = pid;
  }

  // initialize target pose as link's current pose
  this->targetPoses[_link->GetScopedName()] = _pose;

  // initialize command wrench for link to zeros
  this->wrenches[_link->GetScopedName()] = physics::Wrench();

  // Reset Time
  this->prevUpdateTime = this->model->GetWorld()->GetSimTime();
}

/////////////////////////////////////////////////
void LinkController::Reset()
{
  // Reset setpoints and feed-forward.
  this->targetPoses.clear();
  this->posPids.clear();
  this->rotPids.clear();
  this->wrenches.clear();

  // Reset Time
  this->prevUpdateTime = this->model->GetWorld()->GetSimTime();

  // \TODO: Should the PID's be reset as well?
}

/////////////////////////////////////////////////
void LinkController::Update()
{
  common::Time currTime = this->model->GetWorld()->GetSimTime();
  common::Time stepTime = currTime - this->prevUpdateTime;
  this->prevUpdateTime = currTime;

  // Skip the update step if SimTime appears to have gone backward.
  // Negative update time wreaks havok on the integrators.
  // This happens when World::ResetTime is called.
  // TODO: fix this when World::ResetTime is improved
  if (stepTime > 0)
  {
    // compute wrench command updates
    std::map<std::string, math::Pose>::iterator piter;
    for (piter = this->targetPoses.begin(); piter != this->targetPoses.end();
      ++piter)
    {
      math::Pose currentPose = this->links[piter->first]->GetWorldPose();
      math::Pose targetPose = piter->second;
      math::Vector3 errorPos = currentPose.pos - targetPose.pos;

      math::Vector3 errorRot =
        (currentPose.rot * targetPose.rot.GetInverse()).GetAsEuler();

      physics::Wrench *commandWrench = &this->wrenches[piter->first];

      std::vector<common::PID> *posPid = &this->posPids[piter->first];
      commandWrench->force.x = (*posPid)[0].Update(errorPos.x, stepTime);
      commandWrench->force.y = (*posPid)[1].Update(errorPos.y, stepTime);
      commandWrench->force.z = (*posPid)[2].Update(errorPos.z, stepTime);
      std::vector<common::PID> *rotPid = &this->rotPids[piter->first];
      commandWrench->torque.x = (*rotPid)[0].Update(errorRot.x, stepTime);
      commandWrench->torque.y = (*rotPid)[1].Update(errorRot.y, stepTime);
      commandWrench->torque.z = (*rotPid)[2].Update(errorRot.z, stepTime);
    }

    // apply wrench commands to links
    if (!this->wrenches.empty())
    {
      std::map<std::string, physics::Wrench>::iterator iter;
      for (iter = this->wrenches.begin(); iter != this->wrenches.end(); ++iter)
      {
        LinkPtr link = this->links[iter->first];
        if (link)
        {
          link->SetForce(iter->second.force);
          link->SetTorque(iter->second.torque);
        }
      }
    }
  }
  else
    gzwarn << "time step <= 0 in LinkController, Reset simulation?\n";
}

/////////////////////////////////////////////////
void LinkController::OnLinkCmd(ConstLinkCmdPtr &_msg)
{
  std::map<std::string, LinkPtr>::iterator iter;
  iter = this->links.find(_msg->name());
  if (iter != this->links.end())
  {
    if (_msg->has_reset() && _msg->reset())
    {
      if (this->wrenches.find(_msg->name()) != this->wrenches.end())
        this->wrenches.erase(this->wrenches.find(_msg->name()));
    }

    // implement msgs.cc a convert for wrench first
    // if (_msg->has_wrench())
    //   this->wrenches[_msg->name()] = _msg->wrench();

    /*  deal with arrays in proto
    if (_msg->has_pos())
    {
      for (int i = 0; i < 3; ++i)
      {
        // if (_msg->pos()[i].has_target())
        //   this->posPids[_msg->name()][i].SetCmd(_msg->pos()[i].target());
        if (_msg->position().has_p_gain())
          this->posPids[_msg->name()].SetPGain(_msg->position().p_gain());
        if (_msg->position().has_i_gain())
          this->posPids[_msg->name()].SetIGain(_msg->position().i_gain());
        if (_msg->position().has_d_gain())
          this->posPids[_msg->name()].SetDGain(_msg->position().d_gain());
        if (_msg->position().has_i_max())
          this->posPids[_msg->name()].SetIMax(_msg->position().i_max());
        if (_msg->position().has_i_min())
          this->posPids[_msg->name()].SetIMax(_msg->position().i_min());
      }
    }

    if (_msg->has_rot())
    {
      for (int i = 0; i < 3; ++i)
      {
        // if (_msg->rot()[i].has_target())
        //   this->orientations[_msg->name()] = _msg->rot()[i].target();
        if (_msg->rot()[i].has_p_gain())
          this->rotPids[_msg->name()].SetPGain(_msg->rot()[i].p_gain());
        if (_msg->rot()[i].has_i_gain())
          this->rotPids[_msg->name()].SetIGain(_msg->rot()[i].i_gain());
        if (_msg->rot()[i].has_d_gain())
          this->rotPids[_msg->name()].SetDGain(_msg->rot()[i].d_gain());
        if (_msg->rot()[i].has_i_max())
          this->rotPids[_msg->name()].SetIMax(_msg->rot()[i].i_max());
        if (_msg->rot()[i].has_i_min())
          this->rotPids[_msg->name()].SetIMax(_msg->rot()[i].i_min());
      }
    }
    */
  }
  else
    gzerr << "Unable to find link[" << _msg->name() << "]\n";

}
