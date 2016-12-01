/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/AxisVisual.hh"
#include "gazebo/rendering/ArrowVisual.hh"
#include "gazebo/rendering/JointVisualPrivate.hh"
#include "gazebo/rendering/JointVisual.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
JointVisual::JointVisual(const std::string &_name, VisualPtr _vis)
  : Visual(*new JointVisualPrivate, _name, _vis, false)
{
  JointVisualPrivate *dPtr =
      reinterpret_cast<JointVisualPrivate *>(this->dataPtr);
  dPtr->type = VT_PHYSICS;
}

/////////////////////////////////////////////////
JointVisual::~JointVisual()
{
  JointVisualPrivate *dPtr =
      reinterpret_cast<JointVisualPrivate *>(this->dataPtr);

  dPtr->axisVisual.reset();
}

/////////////////////////////////////////////////
void JointVisual::Fini()
{
  JointVisualPrivate *dPtr =
      reinterpret_cast<JointVisualPrivate *>(this->dataPtr);

  if (dPtr->parentAxisVis)
    dPtr->parentAxisVis->Fini();
  dPtr->parentAxisVis.reset();

  Visual::Fini();
}

/////////////////////////////////////////////////
void JointVisual::Load(ConstJointPtr &_msg)
{
  JointVisualPrivate *dPtr =
      reinterpret_cast<JointVisualPrivate *>(this->dataPtr);

  Visual::Load();

  dPtr->axisVisual.reset(
      new AxisVisual(this->GetName() + "_AXIS", shared_from_this()));
  dPtr->axisVisual->Load();

  ignition::math::Pose3d pose;
  if (_msg->has_pose())
    pose = msgs::ConvertIgn(_msg->pose());

  this->SetPosition(pose.Pos());
  this->SetRotation(pose.Rot());

  if (_msg->has_axis2())
  {
    // for hinge2 and universal joints:
    // axis1 is attached to parent link and axis2 is attached to child link

    // create extra joint visual for axis1
    VisualPtr parentVis;
    if (_msg->has_parent() && _msg->parent() == "world")
      parentVis = this->GetScene()->WorldVisual();
    else if (_msg->has_parent_id())
      parentVis = this->GetScene()->GetVisual(_msg->parent_id());

    JointVisualPtr jointVis;
    jointVis.reset(new JointVisual(this->GetName() + "_parent_", parentVis));
    jointVis->Load(_msg, pose + this->GetParent()->GetWorldPose().Ign());

    // attach axis2 to this visual
    msgs::Axis axis2Msg = _msg->axis2();
    dPtr->arrowVisual = this->CreateAxis(msgs::ConvertIgn(axis2Msg.xyz()),
        axis2Msg.use_parent_model_frame(), _msg->type());

    dPtr->parentAxisVis = jointVis;
  }
  else if (_msg->has_axis1())
  {
    // for all other joint types:
    // axis1 is attached to child link
    msgs::Axis axis1Msg = _msg->axis1();
    dPtr->arrowVisual = this->CreateAxis(msgs::ConvertIgn(axis1Msg.xyz()),
        axis1Msg.use_parent_model_frame(), _msg->type());
  }

  // Scale according to the link it is attached to
  double linkSize = std::max(0.1,
      dPtr->parent->GetBoundingBox().GetSize().GetLength());
  dPtr->scaleToLink = ignition::math::Vector3d(linkSize * 0.7,
      linkSize * 0.7, linkSize * 0.7);
  this->SetScale(dPtr->scaleToLink);
  if (dPtr->parentAxisVis)
    dPtr->parentAxisVis->SetScale(dPtr->scaleToLink);

  this->GetSceneNode()->setInheritScale(false);
  this->SetVisibilityFlags(GZ_VISIBILITY_GUI);
}

/////////////////////////////////////////////////
void JointVisual::Load(ConstJointPtr &_msg, const math::Pose &_worldPose)
{
  JointVisualPrivate *dPtr =
      reinterpret_cast<JointVisualPrivate *>(this->dataPtr);

  Visual::Load();

  msgs::Axis axis1Msg = _msg->axis1();
  dPtr->arrowVisual = this->CreateAxis(msgs::ConvertIgn(axis1Msg.xyz()),
      axis1Msg.use_parent_model_frame(), _msg->type());

  // joint pose is always relative to the child link so update axis pose
  this->SetWorldPose(_worldPose);

  this->GetSceneNode()->setInheritScale(false);
  this->SetVisibilityFlags(GZ_VISIBILITY_GUI);
}

/////////////////////////////////////////////////
ArrowVisualPtr JointVisual::CreateAxis(const math::Vector3 &_axis,
    bool _useParentFrame, msgs::Joint::Type _type)
{
  ArrowVisualPtr axis;

  std::stringstream nameStr;
  nameStr << this->GetName() << "_axis_" << this->GetChildCount() << "_AXIS";

  axis.reset(new ArrowVisual(nameStr.str(), shared_from_this()));
  axis->Load();
  axis->SetMaterial("Gazebo/YellowTransparent");

  this->UpdateAxis(axis, _axis, _useParentFrame, _type);

  return axis;
}

/////////////////////////////////////////////////
void JointVisual::UpdateAxis(ArrowVisualPtr _arrowVisual,
    const math::Vector3 &_axis, bool _useParentFrame, msgs::Joint::Type _type)
{
  JointVisualPrivate *dPtr =
      reinterpret_cast<JointVisualPrivate *>(this->dataPtr);

  // Get rotation to axis vector
  math::Vector3 axisDir = _axis;
  math::Vector3 u = axisDir.Normalize();
  math::Vector3 v = math::Vector3::UnitZ;
  double cosTheta = v.Dot(u);
  double angle = acos(cosTheta);
  math::Quaternion quat;
  // check the parallel case
  if (math::equal(angle, M_PI))
    quat.SetFromAxis(u.GetPerpendicular(), angle);
  else
    quat.SetFromAxis((v.Cross(u)).Normalize(), angle);
  _arrowVisual->SetRotation(quat);

  if (_useParentFrame)
  {
    VisualPtr linkVis = this->GetParent();
    ignition::math::Pose3d linkInitPose = linkVis->InitialRelativePose();

    // get rotation of joint visual in model frame
    ignition::math::Quaterniond quatFromModel =
        (this->GetPose().Ign() + linkInitPose).Rot();

    // rotate arrow visual so that the axis vector applies to the model frame.
    _arrowVisual->SetRotation(quatFromModel.Inverse() *
        _arrowVisual->GetRotation().Ign());
  }
  _arrowVisual->ShowRotation(_type == msgs::Joint::REVOLUTE ||
                             _type == msgs::Joint::REVOLUTE2 ||
                             _type == msgs::Joint::UNIVERSAL ||
                             _type == msgs::Joint::GEARBOX);

  if (dPtr->axisVisual)
  {
    _arrowVisual->SetVisible(true);
  }
  else
  {
    return;
  }

  // Hide existing arrow head if it overlaps with the axis
  math::Quaternion axisWorldRotation = _arrowVisual->GetWorldPose().rot;
  math::Quaternion jointWorldRotation = this->GetWorldPose().rot;

  dPtr->axisVisual->ShowAxisHead(0, true);
  dPtr->axisVisual->ShowAxisHead(1, true);
  dPtr->axisVisual->ShowAxisHead(2, true);
  _arrowVisual->ShowShaft(true);

  math::Vector3 axisWorld = axisWorldRotation*math::Vector3::UnitZ;
  if (axisWorld == jointWorldRotation*math::Vector3::UnitX)
  {
    dPtr->axisVisual->ShowAxisHead(0, false);
    _arrowVisual->ShowShaft(false);
  }
  else if (axisWorld == jointWorldRotation*math::Vector3::UnitY)
  {
    dPtr->axisVisual->ShowAxisHead(1, false);
    _arrowVisual->ShowShaft(false);
  }
  else if (axisWorld == jointWorldRotation*math::Vector3::UnitZ)
  {
    dPtr->axisVisual->ShowAxisHead(2, false);
    _arrowVisual->ShowShaft(false);
  }
}

/////////////////////////////////////////////////
void JointVisual::UpdateFromMsg(ConstJointPtr &_msg)
{
  JointVisualPrivate *dPtr =
      reinterpret_cast<JointVisualPrivate *>(this->dataPtr);

  if (_msg->has_pose())
  {
    // Avoid position changing when parent is scaled
    this->SetPosition(msgs::ConvertIgn(_msg->pose().position()) /
        this->GetParent()->GetScale().Ign());
    this->SetRotation(msgs::ConvertIgn(_msg->pose().orientation()));
  }

  ArrowVisualPtr axis2Visual = NULL;
  if (dPtr->parentAxisVis)
  {
    axis2Visual = dPtr->parentAxisVis->GetArrowVisual();
  }

  // Show XYZ heads
  if (dPtr->axisVisual)
  {
    dPtr->axisVisual->ShowAxisHead(0, true);
    dPtr->axisVisual->ShowAxisHead(1, true);
    dPtr->axisVisual->ShowAxisHead(2, true);
  }

  msgs::Axis axis1Msg;
  msgs::Axis axis2Msg;
  // Now has 2 axes
  if (_msg->has_axis2())
  {
    axis1Msg = _msg->axis1();
    axis2Msg = _msg->axis2();
    // Previously already had 2 axes
    if (axis2Visual)
    {
      this->UpdateAxis(dPtr->arrowVisual, msgs::ConvertIgn(axis1Msg.xyz()),
          axis1Msg.use_parent_model_frame(), _msg->type());
      this->UpdateAxis(axis2Visual, msgs::ConvertIgn(axis2Msg.xyz()),
          axis2Msg.use_parent_model_frame(), _msg->type());
      // joint pose is always relative to the child link
      dPtr->parentAxisVis->SetWorldPose(msgs::ConvertIgn(_msg->pose()) +
          this->GetParent()->GetWorldPose().Ign());
    }
    else
    {
      VisualPtr parentVis;
      if (_msg->has_parent() && _msg->parent() == "world")
        parentVis = this->GetScene()->WorldVisual();
      else if (_msg->has_parent_id())
        parentVis = this->GetScene()->GetVisual(_msg->parent_id());

      JointVisualPtr jointVis;
      jointVis.reset(new JointVisual(this->GetName() + "_parent_", parentVis));
      jointVis->Load(_msg,
          msgs::ConvertIgn(_msg->pose()) +
          this->GetParent()->GetWorldPose().Ign());

      dPtr->parentAxisVis = jointVis;
      dPtr->parentAxisVis->SetScale(dPtr->scaleToLink);

      // Previously had 1 axis, which becomes axis 2 now
      if (dPtr->arrowVisual)
      {
        this->UpdateAxis(dPtr->arrowVisual, msgs::ConvertIgn(axis2Msg.xyz()),
            axis2Msg.use_parent_model_frame(), _msg->type());
      }
      // Previously had no axis
      else
      {
        dPtr->arrowVisual = this->CreateAxis(msgs::ConvertIgn(axis2Msg.xyz()),
            axis2Msg.use_parent_model_frame(), _msg->type());
      }
    }
  }
  // Now has 1 axis
  else if (_msg->has_axis1())
  {
    // Hide axis 2
    if (axis2Visual)
      axis2Visual->SetVisible(false);

    axis1Msg = _msg->axis1();
    // Previously had at least 1 axis
    if (dPtr->arrowVisual)
    {
      this->UpdateAxis(dPtr->arrowVisual, msgs::ConvertIgn(axis1Msg.xyz()),
          axis1Msg.use_parent_model_frame(), _msg->type());
    }
    // Previously had no axis
    else
    {
      dPtr->arrowVisual = this->CreateAxis(msgs::ConvertIgn(axis1Msg.xyz()),
          axis1Msg.use_parent_model_frame(), _msg->type());
    }
  }
  // Now has no axis
  else if (_msg->has_type())
  {
    // Hide axes 1 and 2
    if (dPtr->arrowVisual)
      dPtr->arrowVisual->SetVisible(false);
    if (axis2Visual)
      axis2Visual->SetVisible(false);
  }
}

/////////////////////////////////////////////////
JointVisualPtr JointVisual::GetParentAxisVisual() const
{
  JointVisualPrivate *dPtr =
      reinterpret_cast<JointVisualPrivate *>(this->dataPtr);

  return dPtr->parentAxisVis;
}

/////////////////////////////////////////////////
ArrowVisualPtr JointVisual::GetArrowVisual() const
{
  JointVisualPrivate *dPtr =
      reinterpret_cast<JointVisualPrivate *>(this->dataPtr);

  return dPtr->arrowVisual;
}

/////////////////////////////////////////////////
void JointVisual::SetVisible(bool _visible, bool _cascade)
{
  JointVisualPrivate *dPtr =
      reinterpret_cast<JointVisualPrivate *>(this->dataPtr);

  Visual::SetVisible(_visible, _cascade);

  if (dPtr->parentAxisVis)
    dPtr->parentAxisVis->SetVisible(_visible, _cascade);
}
