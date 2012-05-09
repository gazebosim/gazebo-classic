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
  this->joints[_joint->GetName()] = _joint;
}

/////////////////////////////////////////////////
void JointController::Update()
{
  if (this->forces.size() > 0)
  {
    std::map<std::string, double>::iterator iter;
    for (iter = this->forces.begin(); iter != this->forces.end(); ++iter)
      this->joints[iter->first]->SetForce(0, iter->second);
  }

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

/////////////////////////////////////////////////
void JointController::OnJointCmd(ConstJointCmdPtr &_msg)
{
  std::map<std::string, JointPtr>::iterator iter;
  iter = this->joints.find(_msg->name());
  if (iter != this->joints.end())
  {
    if (_msg->has_force())
      this->forces[_msg->name()] = _msg->force();

    if (_msg->has_position())
      this->positions[_msg->name()] = _msg->position();
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
    this->SetJointPosition(iter->second, jiter->second);
  }
}

//////////////////////////////////////////////////
void JointController::SetJointPosition(JointPtr _joint, double _position)
{
  // only deal with hinge and revolute joints in the user
  // request joint_names list
  if (_joint->HasType(Base::HINGE_JOINT) || _joint->HasType(Base::SLIDER_JOINT))
  {
    LinkPtr parentLink = _joint->GetParent();
    LinkPtr childLink = _joint->GetChild();

    if (parentLink && childLink &&
        parentLink->GetName() != childLink->GetName())
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

        this->RotateBodyAndChildren(childLink, anchor, axis, dangle, true);
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

        this->SlideBodyAndChildren(childLink, anchor,
            axis, dposition, true);
      }
      else
      {
        gzwarn << "Setting non HINGE/SLIDER joint types not"
          << "implemented [" << _joint->GetName() << "]\n";
      }
    }
  }

  _joint->SetAngle(0, _position);
}



//////////////////////////////////////////////////
void JointController::RotateBodyAndChildren(LinkPtr _link1,
    const math::Vector3 &_anchor, const math::Vector3 &_axis,
    double _dangle, bool _updateChildren)
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

  // recurse through children bodies
  if (_updateChildren)
  {
    std::vector<LinkPtr> bodies;
    this->GetAllChildrenBodies(bodies, _link1);

    for (std::vector<LinkPtr>::iterator biter = bodies.begin();
        biter != bodies.end(); ++biter)
    {
      this->RotateBodyAndChildren((*biter), _anchor, _axis, _dangle, false);
    }
  }
}

//////////////////////////////////////////////////
void JointController::SlideBodyAndChildren(LinkPtr _link1,
    const math::Vector3 &_anchor, const math::Vector3 &_axis,
    double _dposition, bool _updateChildren)
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

  // recurse through children bodies
  if (_updateChildren)
  {
    std::vector<LinkPtr> bodies;
    this->GetAllChildrenBodies(bodies, _link1);

    for (std::vector<LinkPtr>::iterator biter = bodies.begin();
        biter != bodies.end(); ++biter)
    {
      this->SlideBodyAndChildren((*biter), _anchor, _axis, _dposition, false);
    }
  }
}

//////////////////////////////////////////////////
void JointController::GetAllChildrenBodies(std::vector<LinkPtr> &_bodies,
                                           const LinkPtr &_body)
{
  // strategy, for each child, recursively look for children
  //           for each child, also look for parents to catch multiple roots
  std::map<std::string, JointPtr>::iterator iter;
  for (iter = this->joints.begin(); iter != this->joints.end(); ++iter)
  {
    JointPtr joint = iter->second;

    // recurse through children connected by joints
    LinkPtr parentLink = joint->GetParent();
    LinkPtr childLink = joint->GetChild();
    if (parentLink && childLink
        && parentLink->GetName() != childLink->GetName()
        && parentLink->GetName() == _body->GetName()
        && !this->InBodies(childLink, _bodies))
    {
      _bodies.push_back(childLink);
      this->GetAllChildrenBodies(_bodies, childLink);
      this->GetAllParentBodies(_bodies, childLink, _body);
    }
  }
}

//////////////////////////////////////////////////
void JointController::GetAllParentBodies(std::vector<LinkPtr> &_bodies,
    const LinkPtr &_body, const LinkPtr &_origParentBody)
{
  std::map<std::string, JointPtr>::iterator iter;
  for (iter = this->joints.begin(); iter != this->joints.end(); ++iter)
  {
    JointPtr joint = iter->second;

    // recurse through children connected by joints
    LinkPtr parentLink = joint->GetParent();
    LinkPtr childLink = joint->GetChild();

    if (parentLink && childLink
        && parentLink->GetName() != childLink->GetName()
        && childLink->GetName() == _body->GetName()
        && parentLink->GetName() != _origParentBody->GetName()
        && !this->InBodies(parentLink, _bodies))
    {
      _bodies.push_back(parentLink);
      this->GetAllParentBodies(_bodies, childLink, _origParentBody);
    }
  }
}

//////////////////////////////////////////////////
bool JointController::InBodies(const LinkPtr &_body,
                               const std::vector<LinkPtr> &_bodies)
{
  for (std::vector<LinkPtr>::const_iterator bit = _bodies.begin();
       bit != _bodies.end(); ++bit)
  {
    if ((*bit)->GetName() == _body->GetName())
      return true;
  }

  return false;
}
