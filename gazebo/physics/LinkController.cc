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
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());

  this->linkCmdSub = this->node->Subscribe(std::string("~/") +
      this->model->GetName() + "/link_cmd",
      &LinkController::OnLinkCmd, this);
}

/////////////////////////////////////////////////
void LinkController::AddLink(LinkPtr _link)
{
  this->links[_link->GetScopedName()] = _link;
  for (int i = 0; i < 3; ++i)
  {
    // (*this->posPids[_link->GetScopedName()])[i].Init(1, 0.1, 0.01, 1, -1, 1000, -1000);
    // (*this->rotPids[_link->GetScopedName()])[i].Init(1, 0.1, 0.01, 1, -1, 1000, -1000);
  }
}

/////////////////////////////////////////////////
void LinkController::Reset()
{
  // Reset setpoints and feed-forward.
  this->poses.clear();
  this->posPids.clear();
  this->rotPids.clear();
  this->wrenches.clear();
  // Should the PID's be reset as well?
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
    if (!this->wrenches.empty())
    {
      std::map<std::string, physics::Wrench>::iterator iter;
      for (iter = this->wrenches.begin(); iter != this->wrenches.end(); ++iter)
        this->links[iter->first]->SetForce(iter->second.force);
    }
/*
    if (!this->positions.empty())
    {
      double cmd;
      std::map<std::string, double>::iterator iter;

      for (iter = this->positions.begin(); iter != this->positions.end();
           ++iter)
      {
        cmd = this->posPids[iter->first].Update(
            this->links[iter->first]->GetAngle(0).Radian() - iter->second,
            stepTime);
        this->links[iter->first]->SetForce(0, cmd);
      }
    }

    if (!this->orientations.empty())
    {
      double cmd;
      std::map<std::string, double>::iterator iter;

      for (iter = this->orientations.begin();
           iter != this->orientations.end(); ++iter)
      {
        cmd = this->rotPids[iter->first].Update(
            this->links[iter->first]->GetVelocity(0) - iter->second,
            stepTime);
        this->links[iter->first]->SetForce(0, cmd);
      }
    }
*/
  }
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

//////////////////////////////////////////////////
void LinkController::ForceLinkToPos(LinkPtr _link, math::Vector3 _pos)
{
}

//////////////////////////////////////////////////
void LinkController::ForceLinkToRot(LinkPtr _link, math::Quaternion _rot)
{
}
