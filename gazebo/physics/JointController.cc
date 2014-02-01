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

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Subscriber.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/Joint.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/JointController.hh"
#include "gazebo/physics/PhysicsEngine.hh"

using namespace gazebo;
using namespace physics;

/////////////////////////////////////////////////
JointController::JointController(ModelPtr _model)
  : model(_model)
{
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());

  this->jointCmdSub = this->node->Subscribe(std::string("~/") +
      this->model->GetName() + "/joint_cmd",
      &JointController::OnJointCmd, this);
}

/////////////////////////////////////////////////
void JointController::AddJoint(JointPtr _joint)
{
  this->joints[_joint->GetScopedName()] = _joint;
  this->posPids[_joint->GetScopedName()].Init(1, 0.1, 0.01, 1, -1);
  this->velPids[_joint->GetScopedName()].Init(1, 0.1, 0.01, 1, -1);
}

/////////////////////////////////////////////////
void JointController::Reset()
{
  // Reset setpoints and feed-forward.
  this->positions.clear();
  this->velocities.clear();
  this->forces.clear();
  // Should the PID's be reset as well?
}

/////////////////////////////////////////////////
void JointController::Update()
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
    if (!this->forces.empty())
    {
      std::map<std::string, double>::iterator iter;
      for (iter = this->forces.begin(); iter != this->forces.end(); ++iter)
        this->joints[iter->first]->SetForce(0, iter->second);
    }

    if (!this->positions.empty())
    {
      double cmd;
      std::map<std::string, double>::iterator iter;

      for (iter = this->positions.begin(); iter != this->positions.end();
           ++iter)
      {
        cmd = this->posPids[iter->first].Update(
            this->joints[iter->first]->GetAngle(0).Radian() - iter->second,
            stepTime);
        this->joints[iter->first]->SetForce(0, cmd);
      }
    }

    if (!this->velocities.empty())
    {
      double cmd;
      std::map<std::string, double>::iterator iter;

      for (iter = this->velocities.begin();
           iter != this->velocities.end(); ++iter)
      {
        cmd = this->velPids[iter->first].Update(
            this->joints[iter->first]->GetVelocity(0) - iter->second,
            stepTime);
        this->joints[iter->first]->SetForce(0, cmd);
      }
    }
  }

  // Disabled for now. Collisions don't update properly
  /*if (this->positions.size() > 0)
  {
    std::map<std::string, JointPtr>::iterator iter;
    for (iter = this->joints.begin(); iter != this->joints.end(); ++iter)
    {
      if (this->positions.find(iter->first) == this->positions.end())
        this->positions[iter->first] = iter->second->GetAngle(0).Radian();
    }
    this->SetJointPositions(this->positions);
    this->positions.clear();
  }*/
}

/////////////////////////////////////////////////
void JointController::OnJointCmd(ConstJointCmdPtr &_msg)
{
  std::map<std::string, JointPtr>::iterator iter;
  iter = this->joints.find(_msg->name());
  if (iter != this->joints.end())
  {
    if (_msg->has_reset() && _msg->reset())
    {
      if (this->forces.find(_msg->name()) != this->forces.end())
        this->forces.erase(this->forces.find(_msg->name()));
      if (this->positions.find(_msg->name()) != this->positions.end())
        this->positions.erase(this->positions.find(_msg->name()));
      if (this->velocities.find(_msg->name()) != this->velocities.end())
        this->velocities.erase(this->velocities.find(_msg->name()));
    }

    if (_msg->has_force())
      this->forces[_msg->name()] = _msg->force();


    if (_msg->has_position())
    {
      if (_msg->position().has_target())
        this->positions[_msg->name()] = _msg->position().target();
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

    if (_msg->has_velocity())
    {
      if (_msg->velocity().has_target())
        this->velocities[_msg->name()] = _msg->velocity().target();
      if (_msg->velocity().has_p_gain())
        this->velPids[_msg->name()].SetPGain(_msg->velocity().p_gain());
      if (_msg->velocity().has_i_gain())
        this->velPids[_msg->name()].SetIGain(_msg->velocity().i_gain());
      if (_msg->velocity().has_d_gain())
        this->velPids[_msg->name()].SetDGain(_msg->velocity().d_gain());
      if (_msg->velocity().has_i_max())
        this->velPids[_msg->name()].SetIMax(_msg->velocity().i_max());
      if (_msg->velocity().has_i_min())
        this->velPids[_msg->name()].SetIMax(_msg->velocity().i_min());
    }
  }
  else
    gzerr << "Unable to find joint[" << _msg->name() << "]\n";
}

//////////////////////////////////////////////////
void JointController::SetJointPosition(const std::string &_name,
                                       double _position, int _index)
{
  std::map<std::string, JointPtr>::iterator jiter = this->joints.find(_name);
  if (jiter != this->joints.end())
    this->SetJointPosition(jiter->second, _position, _index);
  else
    gzwarn << "SetJointPosition [" << _name << "] not found\n";
}

//////////////////////////////////////////////////
void JointController::SetJointPositions(
    const std::map<std::string, double> &_jointPositions)
{
  // go through all joints in this model and update each one
  //   for each joint update, recursively update all children
  std::map<std::string, JointPtr>::iterator iter;
  std::map<std::string, double>::const_iterator jiter;

  for (iter = this->joints.begin(); iter != this->joints.end(); ++iter)
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
  // truncate position by joint limits
  double lower = _joint->GetLowStop(_index).Radian();
  double upper = _joint->GetHighStop(_index).Radian();
  _position = _position < lower? lower : (_position > upper? upper : _position);

  // keep track of updatd links, make sure each is upated only once
  this->updatedLinks.clear();

  // only deal with hinge and revolute joints in the user
  // request joint_names list
  if (_joint->HasType(Base::HINGE_JOINT) ||
      _joint->HasType(Base::UNIVERSAL_JOINT) ||
      _joint->HasType(Base::SLIDER_JOINT))
  {
    LinkPtr parentLink = _joint->GetParent();
    LinkPtr childLink = _joint->GetChild();

    if ((!parentLink && childLink) ||
        (parentLink && childLink &&
         parentLink->GetScopedName() != childLink->GetScopedName()))
    {
      // transform about the current anchor, about the axis
      // rotate child (childLink) about anchor point, by delta-angle
      // along axis
      double dposition = _position - _joint->GetAngle(_index).Radian();

      math::Vector3 anchor;
      math::Vector3 axis;

      if (this->model->IsStatic())
      {
        math::Pose linkWorldPose = childLink->GetWorldPose();
        /// \TODO: we want to get axis in global frame, but GetGlobalAxis
        /// not implemented for static models yet.
        axis = linkWorldPose.rot.RotateVector(_joint->GetLocalAxis(_index));
        anchor = linkWorldPose.pos;
      }
      else
      {
        anchor = _joint->GetAnchor(_index);
        axis = _joint->GetGlobalAxis(_index);
      }

      // we don't want to move the parent link
      if (parentLink)
        this->updatedLinks.push_back(parentLink);

      this->MoveLinks(_joint, childLink, anchor, axis, dposition,
        true);
    }
  }

  /// @todo:  Set link and joint "velocities" based on change / time
}



//////////////////////////////////////////////////
void JointController::MoveLinks(JointPtr _joint, LinkPtr _link,
    const math::Vector3 &_anchor, const math::Vector3 &_axis,
    double _dposition, bool _updateChildren)
{
  if (!this->ContainsLink(this->updatedLinks, _link))
  {
    if (_joint->HasType(Base::HINGE_JOINT) ||
        _joint->HasType(Base::UNIVERSAL_JOINT))
    {
      math::Pose linkWorldPose = _link->GetWorldPose();

      // relative to anchor point
      math::Pose relativePose(linkWorldPose.pos - _anchor, linkWorldPose.rot);

      // take axis rotation and turn it int a quaternion
      math::Quaternion rotation(_axis, _dposition);

      // rotate relative pose by rotation
      math::Pose newRelativePose;

      newRelativePose.pos = rotation.RotateVector(relativePose.pos);
      newRelativePose.rot = rotation * relativePose.rot;

      math::Pose newWorldPose(newRelativePose.pos + _anchor,
                              newRelativePose.rot);

      _link->SetWorldPose(newWorldPose);

      /// \TODO: ideally we want to set this according to
      /// Joint Trajectory velocity and use time step since last update.
      /// double dt =
      /// this->model->GetWorld()->GetPhysicsEngine()->GetMaxStepTime();
      /// this->ComputeAndSetLinkTwist(_link, newWorldPose, newWorldPose, dt);

      this->updatedLinks.push_back(_link);
    }
    else if (_joint->HasType(Base::SLIDER_JOINT))
    {
      math::Pose linkWorldPose = _link->GetWorldPose();

      // relative to anchor point
      math::Pose relativePose(linkWorldPose.pos - _anchor, linkWorldPose.rot);

      // slide relative pose by dposition along axis
      math::Pose newRelativePose;
      newRelativePose.pos = relativePose.pos + _axis * _dposition;
      newRelativePose.rot = relativePose.rot;

      math::Pose newWorldPose(newRelativePose.pos + _anchor,
                              newRelativePose.rot);

      _link->SetWorldPose(newWorldPose);

      /// \TODO: ideally we want to set this according to Joint Trajectory
      /// velocity and use time step since last update.
      /// double dt = this->model->GetWorld()->GetPhysicsEngine()->
      /// GetMaxStepTime();
      /// this->ComputeAndSetLinkTwist(_link, newWorldPose, newWorldPose, dt);

      this->updatedLinks.push_back(_link);
    }
    else
      gzerr << "should not be here\n";
  }


  // recurse through connected links
  if (_updateChildren)
  {
    Link_V connected_links;
    this->AddConnectedLinks(connected_links, _link, true);

    for (Link_V::iterator liter = connected_links.begin();
        liter != connected_links.end(); ++liter)
    {
      this->MoveLinks(_joint, (*liter), _anchor, _axis, _dposition);
    }
  }
}

//////////////////////////////////////////////////
void JointController::ComputeAndSetLinkTwist(LinkPtr _link,
     const math::Pose &_old, const math::Pose &_new, double _dt)
{
    math::Vector3 linear_vel(0, 0, 0);
    math::Vector3 angular_vel(0, 0, 0);
    if (math::equal(_dt, 0.0))
    {
      gzwarn << "dt is 0, unable to compute velocity, set to 0s\n";
    }
    else
    {
      linear_vel = (_new.pos - _old.pos) / _dt;
      angular_vel = (_new.rot.GetAsEuler() - _old.rot.GetAsEuler()) / _dt;
    }
    _link->SetLinearVel(linear_vel);
    _link->SetAngularVel(angular_vel);
}

//////////////////////////////////////////////////
void JointController::AddConnectedLinks(Link_V &_linksOut,
                                        const LinkPtr &_link,
                                        bool _checkParentTree)
{
  // strategy, for each child, recursively look for children
  //           for each child, also look for parents to catch multiple roots


  Link_V childLinks = _link->GetChildJointsLinks();
  for (Link_V::iterator childLink = childLinks.begin();
                                      childLink != childLinks.end();
                                      ++childLink)
  {
    // add this link to the list of links to be updated by SetJointPosition
    if (!this->ContainsLink(_linksOut, *childLink))
    {
      _linksOut.push_back(*childLink);
      // recurse into children, but not parents
      this->AddConnectedLinks(_linksOut, *childLink);
    }

    if (_checkParentTree)
    {
      // catch additional roots by looping
      // through all parents of childLink,
      // but skip parent link itself (_link)
      Link_V parentLinks = (*childLink)->GetParentJointsLinks();
      for (Link_V::iterator parentLink = parentLinks.begin();
                                          parentLink != parentLinks.end();
                                          ++parentLink)
      {
        if ((*parentLink)->GetName() != _link->GetName() &&
            !this->ContainsLink(_linksOut, (*parentLink)))
        {
          _linksOut.push_back(*parentLink);
          // add all childrend links of parentLink, but
          // stop the recursion if any of the child link is already added
          this->AddConnectedLinks(_linksOut, *parentLink, _link);
        }
      }
    }
  }
}
