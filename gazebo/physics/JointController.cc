/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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

#include "transport/Node.hh"
#include "transport/Subscriber.hh"
#include "physics/Model.hh"
#include "physics/World.hh"
#include "physics/Joint.hh"
#include "physics/Link.hh"
#include "physics/JointController.hh"

using namespace gazebo;
using namespace physics;

////////////////////////////////////////////////////////////////////////////////
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
  this->joints[_joint->GetName()] = _joint;
  this->posPids[_joint->GetName()].Init(1, 0.1, 0.01, 1, -1);
  this->velPids[_joint->GetName()].Init(1, 0.1, 0.01, 1, -1);
}

/////////////////////////////////////////////////
void JointController::Reset()
{
  this->positions.clear();
  this->velocities.clear();
  this->forces.clear();
}

/////////////////////////////////////////////////
void JointController::Update()
{
  common::Time currTime = this->model->GetWorld()->GetSimTime();
  common::Time stepTime = currTime - this->prevUpdateTime;
  this->prevUpdateTime = currTime;

  if (this->forces.size() > 0)
  {
    std::map<std::string, double>::iterator iter;
    for (iter = this->forces.begin(); iter != this->forces.end(); ++iter)
      this->joints[iter->first]->SetForce(0, iter->second);
  }

  /*
  if (this->positions.size() > 0)
  {
    double cmd;
    std::map<std::string, double>::iterator iter;

    for (iter = this->positions.begin(); iter != this->positions.end(); ++iter)
    {
      cmd = this->posPids[iter->first].Update(
          this->joints[iter->first]->GetAngle(0).GetAsRadian() - iter->second,
          stepTime);
      this->joints[iter->first]->SetForce(0, cmd);
    }
  }*/

  if (this->velocities.size() > 0)
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

  // Disabled for now. Collisions don't update properly
  if (this->positions.size() > 0)
  {
    std::map<std::string, JointPtr>::iterator iter;
    for (iter = this->joints.begin(); iter != this->joints.end(); ++iter)
    {
      if (this->positions.find(iter->first) == this->positions.end())
        this->positions[iter->first] = iter->second->GetAngle(0).GetAsRadian();
    }
    this->SetJointPositions(this->positions);
    this->positions.clear();
  }
}

////////////////////////////////////////////////////////////////////////////////
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
                                       double _position)
{
  if (this->joints.find(_name) != this->joints.end())
    this->SetJointPosition(this->joints[_name], _position);
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
    jiter = _jointPositions.find(iter->second->GetName());
    if (jiter != _jointPositions.end())
      this->SetJointPosition(iter->second, jiter->second);
  }
}

//////////////////////////////////////////////////
void JointController::SetJointPosition(JointPtr _joint, double _position)
{
  // truncate position by joint limits
  double lower = _joint->GetLowStop(0).GetAsRadian();
  double upper = _joint->GetHighStop(0).GetAsRadian();
  _position = _position < lower? lower :
    (_position > upper? upper : _position);

  // keep track of updatd links, make sure each is upated only once
  this->updated_links.clear();

  // only deal with hinge and revolute joints in the user
  // request joint_names list
  if (_joint->HasType(Base::HINGE_JOINT) || _joint->HasType(Base::SLIDER_JOINT))
  {
    LinkPtr parentLink = _joint->GetParent();
    LinkPtr childLink = _joint->GetChild();

    if ((!parentLink && childLink) ||
        (parentLink && childLink &&
         parentLink->GetName() != childLink->GetName()))
    {
      // transform about the current anchor, about the axis
      if (_joint->HasType(Base::HINGE_JOINT))
      {
        // rotate child (childLink) about anchor point, by delta-angle
        // along axis
        double dangle = _position - _joint->GetAngle(0).GetAsRadian();

        math::Vector3 anchor;
        math::Vector3 axis;

        if (this->model->IsStatic())
        {
          math::Pose linkWorldPose = childLink->GetWorldPose();
          axis = linkWorldPose.rot.RotateVector(_joint->GetLocalAxis(0));
          anchor = linkWorldPose.pos;
        }
        else
        {
          anchor = _joint->GetAnchor(0);
          axis = _joint->GetGlobalAxis(0);
        }

        this->RotateLinkAndChildren(childLink, anchor, axis, dangle, true);
      }
      else if (_joint->HasType(Base::SLIDER_JOINT))
      {
        double dposition = _position - _joint->GetAngle(0).GetAsRadian();

        math::Vector3 anchor;
        math::Vector3 axis;

        if (this->model->IsStatic())
        {
          math::Pose linkWorldPose = childLink->GetWorldPose();
          axis = linkWorldPose.rot.RotateVector(_joint->GetLocalAxis(0));
          anchor = linkWorldPose.pos;
        }
        else
        {
          anchor = _joint->GetAnchor(0);
          axis = _joint->GetGlobalAxis(0);
        }

        this->SlideLinkAndChildren(childLink, anchor,
            axis, dposition, true);
      }
      else
      {
        gzwarn << "Setting non HINGE/SLIDER joint types not"
          << "implemented [" << _joint->GetName() << "]\n";
      }
    }
  }

  /// @todo:  Set link and joint "velocities" based on change / time
}



//////////////////////////////////////////////////
void JointController::RotateLinkAndChildren(LinkPtr _link1,
    const math::Vector3 &_anchor, const math::Vector3 &_axis,
    double _dangle, bool _updateChildren)
{
  if (!this->ContainsLink( this->updated_links, _link1))
  {
    math::Pose linkWorldPose = _link1->GetWorldPose();

    // relative to anchor point
    math::Pose relativePose(linkWorldPose.pos - _anchor, linkWorldPose.rot);

    // take axis rotation and turn it int a quaternion
    math::Quaternion rotation(_axis, _dangle);

    // rotate relative pose by rotation
    math::Pose newRelativePose;

    newRelativePose.pos = rotation.RotateVector(relativePose.pos);
    newRelativePose.rot = rotation * relativePose.rot;

    math::Pose newWorldPose(newRelativePose.pos + _anchor,
                            newRelativePose.rot);

    _link1->SetWorldPose(newWorldPose);
    _link1->SetLinearVel(math::Vector3(0, 0, 0));
    _link1->SetAngularVel(math::Vector3(0, 0, 0));

    this->updated_links.push_back(_link1);
  }

  // recurse through children links
  if (_updateChildren)
  {
    std::vector<LinkPtr> links;
    this->AddConnectedLinks(links, _link1);

    for (std::vector<LinkPtr>::iterator biter = links.begin();
        biter != links.end(); ++biter)
    {
      this->RotateLinkAndChildren((*biter), _anchor, _axis, _dangle, false);
    }
  }
}

//////////////////////////////////////////////////
void JointController::SlideLinkAndChildren(LinkPtr _link1,
    const math::Vector3 &_anchor, const math::Vector3 &_axis,
    double _dposition, bool _updateChildren)
{
  if (!this->ContainsLink(this->updated_links, _link1))
  {
    math::Pose linkWorldPose = _link1->GetWorldPose();

    // relative to anchor point
    math::Pose relativePose(linkWorldPose.pos - _anchor, linkWorldPose.rot);

    // slide relative pose by dposition along axis
    math::Pose newRelativePose;
    newRelativePose.pos = relativePose.pos + _axis * _dposition;
    newRelativePose.rot = relativePose.rot;

    math::Pose newWorldPose(newRelativePose.pos + _anchor, newRelativePose.rot);

    _link1->SetWorldPose(newWorldPose);
    _link1->SetLinearVel(math::Vector3(0, 0, 0));
    _link1->SetAngularVel(math::Vector3(0, 0, 0));

    this->updated_links.push_back(_link1);
  }

  // recurse through children links
  if (_updateChildren)
  {
    std::vector<LinkPtr> links;
    this->AddConnectedLinks(links, _link1);

    for (std::vector<LinkPtr>::iterator biter = links.begin();
        biter != links.end(); ++biter)
    {
      this->SlideLinkAndChildren((*biter), _anchor, _axis, _dposition, false);
    }
  }
}

//////////////////////////////////////////////////
void JointController::AddConnectedLinks(std::vector<LinkPtr> &_links_out,
                                           const LinkPtr &_current_link)
{
  // strategy, for each child, recursively look for children
  //           for each child, also look for parents to catch multiple roots


  std::vector<LinkPtr> childLinks = _current_link->GetChildLinks();
  for (std::vector<LinkPtr>::iterator childLink = childLinks.begin();
                                      childLink != childLinks.end();
                                      ++childLink)
  {
    // add this link to the list of links to be updated by SetJointPosition
    if (!this->ContainsLink(_links_out, *childLink))
    {
      _links_out.push_back(*childLink);
      this->AddConnectedLinks(_links_out, *childLink);
    }

    // loop through all parents of children, but make sure
    // the parent link is not self (_current_link)
    std::vector<LinkPtr> parentLinks = _current_link->GetParentLinks();
    for (std::vector<LinkPtr>::iterator parentLink = parentLinks.begin();
                                        parentLink != parentLinks.end();
                                        ++parentLink)
    {
      if ((*parentLink)->GetName() != _current_link->GetName())
      {
        // add all childrend links of parentLinks, but
        // stop the recursion if any of the child link is already added
        this->AddChildLinks(_links_out, *parentLink, _current_link);
      }
    }
  }
}

//////////////////////////////////////////////////
/// Add child of 
void JointController::AddChildLinks(std::vector<LinkPtr> &_links_out,
    const LinkPtr &_current_link, const LinkPtr &_exclude_link)
{
  std::vector<LinkPtr> childLinks = _current_link->GetChildLinks();
  for (std::vector<LinkPtr>::iterator childLink = childLinks.begin();
                                      childLink != childLinks.end();
                                      ++childLink)
  {
    // add this link to the list of links to be updated by SetJointPosition
    if ((*childLink)->GetName() != _exclude_link->GetName() &&
        !this->ContainsLink(_links_out, *childLink))
    {
      _links_out.push_back(*childLink);
      this->AddConnectedLinks(_links_out, *childLink);
    }
  }
/*
  std::map<std::string, JointPtr>::iterator iter;
  for (iter = this->joints.begin(); iter != this->joints.end(); ++iter)
  {
    JointPtr joint = iter->second;

    // recurse through children connected by joints
    LinkPtr parentLink = joint->GetParent();
    LinkPtr childLink = joint->GetChild();

    if (parentLink)
    {
    if (childLink
        && parentLink->GetName() != childLink->GetName()
        && childLink->GetName() == _current_link->GetName()
        && parentLink->GetName() != _exclude_link->GetName()
        && !this->ContainsLink(_links_out, parentLink))
    {
      _links_out.push_back(parentLink);
      this->AddParentLinks(_links_out, childLink, _exclude_link);
    }
    }
    else // no parent, check child only
    {
    }
  }
*/
}
