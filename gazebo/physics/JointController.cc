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

  // Disabled for now. Collisions don't update properly
  // if (this->dataPtr->positions.size() > 0)
  // {
  //   std::map<std::string, JointPtr>::iterator iter;
  //   for (iter = this->dataPtr->joints.begin();
  //        iter != this->dataPtr->joints.end(); ++iter)
  //   {
  //     if (this->dataPtr->positions.find(iter->first) ==
  //         this->dataPtr->positions.end())
  //     {
  //       this->dataPtr->positions[iter->first] =
  //         iter->second->GetAngle(0).Radian();
  //     }
  //   }
  //   this->SetJointPositions(this->dataPtr->positions);
  //   this->dataPtr->positions.clear();
  // }
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
void JointController::SetJointPosition(const std::string & /*_name*/,
                                       double /*_position*/, int /*_index*/)
{
  gzwarn << "Setting joint position is disabled, see issue #1138\n";
  return;

  /// std::map<std::string, JointPtr>::iterator jiter =
  ///   this->dataPtr->joints.find(_name);

  /// if (jiter != this->dataPtr->joints.end())
  ///   this->SetJointPosition(jiter->second, _position, _index);
  /// else
  ///   gzwarn << "SetJointPosition [" << _name << "] not found\n";
}

//////////////////////////////////////////////////
void JointController::SetJointPositions(
    const std::map<std::string, double> & /*_jointPositions*/)
{
  gzwarn << "Setting joint positions is disabled, see issue #1138\n";
  return;

  // go through all joints in this model and update each one
  //   for each joint update, recursively update all children
  // std::map<std::string, JointPtr>::iterator iter;
  // std::map<std::string, double>::const_iterator jiter;

  // for (iter = this->dataPtr->joints.begin();
  //     iter != this->dataPtr->joints.end(); ++iter)
  // {
  //   // First try name without scope, i.e. joint_name
  //   jiter = _jointPositions.find(iter->second->GetName());

  //   if (jiter == _jointPositions.end())
  //   {
  //     // Second try name with scope, i.e. model_name::joint_name
  //     jiter = _jointPositions.find(iter->second->GetScopedName());
  //     if (jiter == _jointPositions.end())
  //       continue;
  //   }

  //   this->SetJointPosition(iter->second, jiter->second);
  // }
}

//////////////////////////////////////////////////
void JointController::SetJointPosition(
  JointPtr /*_joint*/, double /*_position*/, int /*_index*/)
{
  gzwarn << "Setting joint position is disabled, see issue #1138\n";
  return;

  // truncate position by joint limits
  // double lower = _joint->GetLowStop(_index).Radian();
  // double upper = _joint->GetHighStop(_index).Radian();
  // _position = _position < lower? lower :
  // (_position > upper? upper : _position);

  // // keep track of updatd links, make sure each is upated only once
  // this->dataPtr->updatedLinks.clear();

  // // only deal with hinge and revolute joints in the user
  // // request joint_names list
  // if (_joint->HasType(Base::HINGE_JOINT) ||
  //     _joint->HasType(Base::UNIVERSAL_JOINT) ||
  //     _joint->HasType(Base::SLIDER_JOINT))
  // {
  //   LinkPtr parentLink = _joint->GetParent();
  //   LinkPtr childLink = _joint->GetChild();

  //   if ((!parentLink && childLink) ||
  //       (parentLink && childLink &&
  //        parentLink->GetScopedName() != childLink->GetScopedName()))
  //   {
  //     // transform about the current anchor, about the axis
  //     // rotate child (childLink) about anchor point, by delta-angle
  //     // along axis
  //     double dposition = _position - _joint->GetAngle(_index).Radian();

  //     math::Vector3 anchor;
  //     math::Vector3 axis;

  //     if (this->dataPtr->model->IsStatic())
  //     {
  //       math::Pose linkWorldPose = childLink->GetWorldPose();
  //       /// \TODO: we want to get axis in global frame, but GetGlobalAxis
  //       /// not implemented for static models yet.
  //       axis = linkWorldPose.rot.RotateVector(_joint->GetLocalAxis(_index));
  //       anchor = linkWorldPose.pos;
  //     }
  //     else
  //     {
  //       anchor = _joint->GetAnchor(_index);
  //       axis = _joint->GetGlobalAxis(_index);
  //     }

  //     // we don't want to move the parent link
  //     if (parentLink)
  //       this->dataPtr->updatedLinks.push_back(parentLink);

  //     this->MoveLinks(_joint, childLink, anchor, axis, dposition,
  //       true);
  //   }
  // }

  /// \todo:  Set link and joint "velocities" based on change / time
}

//////////////////////////////////////////////////
// void JointController::MoveLinks(JointPtr _joint, LinkPtr _link,
//     const math::Vector3 &_anchor, const math::Vector3 &_axis,
//     double _dposition, bool _updateChildren)
// {
//   if (!this->ContainsLink(this->dataPtr->updatedLinks, _link))
//   {
//     if (_joint->HasType(Base::HINGE_JOINT) ||
//         _joint->HasType(Base::UNIVERSAL_JOINT))
//     {
//       math::Pose linkWorldPose = _link->GetWorldPose();
//
//       // relative to anchor point
//       math::Pose relativePose(linkWorldPose.pos - _anchor,
//                               linkWorldPose.rot);
//
//       // take axis rotation and turn it int a quaternion
//       math::Quaternion rotation(_axis, _dposition);
//
//       // rotate relative pose by rotation
//       math::Pose newRelativePose;
//
//       newRelativePose.pos = rotation.RotateVector(relativePose.pos);
//       newRelativePose.rot = rotation * relativePose.rot;
//
//       math::Pose newWorldPose(newRelativePose.pos + _anchor,
//                               newRelativePose.rot);
//
//       _link->SetWorldPose(newWorldPose);
//
//       // \TODO: ideally we want to set this according to
//       // Joint Trajectory velocity and use time step since last update.
//       // double dt =
//       // this->dataPtr->model->GetWorld()->GetPhysicsEngine()->
//               GetMaxStepTime();
//       // this->ComputeAndSetLinkTwist(_link, newWorldPose,
//                                       newWorldPose, dt);
//
//       this->dataPtr->updatedLinks.push_back(_link);
//     }
//     else if (_joint->HasType(Base::SLIDER_JOINT))
//     {
//       math::Pose linkWorldPose = _link->GetWorldPose();
//
//       // relative to anchor point
//       math::Pose relativePose(linkWorldPose.pos - _anchor,
//                               linkWorldPose.rot);
//
//       // slide relative pose by dposition along axis
//       math::Pose newRelativePose;
//       newRelativePose.pos = relativePose.pos + _axis * _dposition;
//       newRelativePose.rot = relativePose.rot;
//
//       math::Pose newWorldPose(newRelativePose.pos + _anchor,
//                               newRelativePose.rot);
//
//       _link->SetWorldPose(newWorldPose);
//
//       /// \TODO: ideally we want to set this according to Joint Trajectory
//       /// velocity and use time step since last update.
//       /// double dt = this->dataPtr->model->GetWorld()->GetPhysicsEngine()->
//       /// GetMaxStepTime();
//       /// this->ComputeAndSetLinkTwist(_link, newWorldPose,
//                                        newWorldPose, dt);
//
//       this->dataPtr->updatedLinks.push_back(_link);
//     }
//     else
//       gzerr << "should not be here\n";
//   }
//
//
//   // recurse through connected links
//   if (_updateChildren)
//   {
//     Link_V connected_links;
//     this->AddConnectedLinks(connected_links, _link, true);
//
//     for (Link_V::iterator liter = connected_links.begin();
//         liter != connected_links.end(); ++liter)
//     {
//       this->MoveLinks(_joint, (*liter), _anchor, _axis, _dposition);
//     }
//   }
// }

//////////////////////////////////////////////////
// void JointController::ComputeAndSetLinkTwist(LinkPtr _link,
//      const math::Pose &_old, const math::Pose &_new, double _dt)
// {
//     math::Vector3 linear_vel(0, 0, 0);
//     math::Vector3 angular_vel(0, 0, 0);
//     if (math::equal(_dt, 0.0))
//     {
//       gzwarn << "dt is 0, unable to compute velocity, set to 0s\n";
//     }
//     else
//     {
//       linear_vel = (_new.pos - _old.pos) / _dt;
//       angular_vel = (_new.rot.GetAsEuler() - _old.rot.GetAsEuler()) / _dt;
//     }
//     _link->SetLinearVel(linear_vel);
//     _link->SetAngularVel(angular_vel);
// }

//////////////////////////////////////////////////
// void JointController::AddConnectedLinks(Link_V &_linksOut,
//                                         const LinkPtr &_link,
//                                         bool _checkParentTree)
// {
//   // strategy, for each child, recursively look for children
//   //           for each child, also look for parents to catch multiple roots
//
//
//   Link_V childLinks = _link->GetChildJointsLinks();
//   for (Link_V::iterator childLink = childLinks.begin();
//                                       childLink != childLinks.end();
//                                       ++childLink)
//   {
//     // add this link to the list of links to be updated by SetJointPosition
//     if (!this->ContainsLink(_linksOut, *childLink))
//     {
//       _linksOut.push_back(*childLink);
//       // recurse into children, but not parents
//       this->AddConnectedLinks(_linksOut, *childLink);
//     }
//
//     if (_checkParentTree)
//     {
//       // catch additional roots by looping
//       // through all parents of childLink,
//       // but skip parent link itself (_link)
//       Link_V parentLinks = (*childLink)->GetParentJointsLinks();
//       for (Link_V::iterator parentLink = parentLinks.begin();
//                                           parentLink != parentLinks.end();
//                                           ++parentLink)
//       {
//         if ((*parentLink)->GetName() != _link->GetName() &&
//             !this->ContainsLink(_linksOut, (*parentLink)))
//         {
//           _linksOut.push_back(*parentLink);
//           // add all childrend links of parentLink, but
//           // stop the recursion if any of the child link is already added
//           this->AddConnectedLinks(_linksOut, *parentLink, _link);
//         }
//       }
//     }
//   }
// }

/////////////////////////////////////////////////
/// bool JointController::ContainsLink(
/// const Link_V &_vector, const LinkPtr &_value)
/// {
///   for (Link_V::const_iterator iter = _vector.begin();
///        iter != _vector.end(); ++iter)
///   {
///     if ((*iter)->GetScopedName() == _value->GetScopedName())
///       return true;
///   }
///   return false;
/// }

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
