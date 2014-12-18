/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
}

/////////////////////////////////////////////////
JointVisual::~JointVisual()
{
  JointVisualPrivate *dPtr =
      reinterpret_cast<JointVisualPrivate *>(this->dataPtr);

  dPtr->axisVisual.reset();
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

  this->SetPosition(msgs::Convert(_msg->pose().position()));
  this->SetRotation(msgs::Convert(_msg->pose().orientation()));

  if (_msg->has_axis2())
  {
    // for hinge2 and universal joints:
    // axis1 is attached to parent link and axis2 is attached to child link

    // create extra joint visual for axis1
    VisualPtr parentVis;
    if (_msg->has_parent() && _msg->parent() == "world")
      parentVis = this->GetScene()->GetWorldVisual();
    else if (_msg->has_parent_id())
      parentVis = this->GetScene()->GetVisual(_msg->parent_id());
    else
      parentVis = this->GetScene()->GetWorldVisual();

    JointVisualPtr jointVis;
    jointVis.reset(new JointVisual(this->GetName() + "_parent_", parentVis));
    jointVis->Load(_msg,
        msgs::Convert(_msg->pose()) + this->GetParent()->GetWorldPose());

    // attach axis2 to this visual
    msgs::Axis axis2Msg = _msg->axis2();
    dPtr->arrow2Visual = this->CreateAxisVisual(msgs::Convert(axis2Msg.xyz()),
        axis2Msg.use_parent_model_frame(), _msg->type());

    dPtr->parentAxisVis = jointVis;
  }
  else if (_msg->has_axis1())
  {
    // for all other joint types:
    // axis1 is attached to child link
    msgs::Axis axis1Msg = _msg->axis1();
    dPtr->arrow1Visual = this->CreateAxisVisual(msgs::Convert(axis1Msg.xyz()),
        axis1Msg.use_parent_model_frame(), _msg->type());
  }

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
  dPtr->arrow1Visual = this->CreateAxisVisual(msgs::Convert(axis1Msg.xyz()),
      axis1Msg.use_parent_model_frame(), _msg->type());

  // joint pose is always relative to the child link so update axis pose
  this->SetWorldPose(_worldPose);

  this->GetSceneNode()->setInheritScale(false);
  this->SetVisibilityFlags(GZ_VISIBILITY_GUI);
}

/////////////////////////////////////////////////
ArrowVisualPtr JointVisual::CreateAxisVisual(const math::Vector3 &_axis, bool _useParentFrame,
    msgs::Joint::Type _type)
{
  JointVisualPrivate *dPtr =
      reinterpret_cast<JointVisualPrivate *>(this->dataPtr);

  ArrowVisualPtr axis;

  std::stringstream nameStr;
  nameStr << this->GetName() << "_axis_" << this->GetChildCount() << "_AXIS";

  axis.reset(new ArrowVisual(nameStr.str(), shared_from_this()));
  axis->Load();
  axis->SetMaterial("Gazebo/YellowTransparent");

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
  axis->SetRotation(quat);

  if (_useParentFrame)
  {
    // if set to use parent model frame
    // rotate the arrow visual relative to the model
    VisualPtr model = this->GetRootVisual();
    math::Quaternion quatFromModel =
        model->GetWorldPose().rot.GetInverse()*this->GetWorldPose().rot;
    axis->SetRotation(quatFromModel.GetInverse()*axis->GetRotation());
  }
  if (_type == msgs::Joint::REVOLUTE || _type == msgs::Joint::REVOLUTE2
      || _type == msgs::Joint::UNIVERSAL || _type == msgs::Joint::GEARBOX)
    axis->ShowRotation(true);

  math::Quaternion axisWorldRotation = axis->GetWorldPose().rot;
  math::Quaternion jointWorldRotation = this->GetWorldPose().rot;

  // hide the existing axis's arrow head if it overlaps with the one we are
  // creating
  math::Vector3 axisWorld = axisWorldRotation*math::Vector3::UnitZ;
  if (axisWorld == jointWorldRotation*math::Vector3::UnitX)
  {
    if (dPtr->axisVisual)
    {
      dPtr->axisVisual->ShowAxisHead(0, false);
      axis->ShowShaft(false);
    }
  }
  else if (axisWorld == jointWorldRotation*math::Vector3::UnitY)
  {
    if (dPtr->axisVisual)
    {
      dPtr->axisVisual->ShowAxisHead(1, false);
      axis->ShowShaft(false);
    }
  }
  else if (axisWorld == jointWorldRotation*math::Vector3::UnitZ)
  {
    if (dPtr->axisVisual)
    {
      dPtr->axisVisual->ShowAxisHead(2, false);
      axis->ShowShaft(false);
    }
  }

  return axis;
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

/////////////////////////////////////////////////
void JointVisual::UpdateFromMsg(ConstJointPtr &_msg)
{
  JointVisualPrivate *dPtr =
      reinterpret_cast<JointVisualPrivate *>(this->dataPtr);

  if (_msg->has_pose())
  {
    this->SetPosition(msgs::Convert(_msg->pose().position()));
    this->SetRotation(msgs::Convert(_msg->pose().orientation()));
  }

  if (dPtr->arrow1Visual)
    dPtr->arrow1Visual->SetVisible(false);
  if (_msg->has_axis1())
  {
    msgs::Axis axis1Msg = _msg->axis1();
    if (dPtr->arrow1Visual)
    {
      this->UpdateAxisVisual(dPtr->arrow1Visual, msgs::Convert(axis1Msg.xyz()),
          axis1Msg.use_parent_model_frame(), _msg->type());
    }
    else
    {
      dPtr->arrow1Visual = this->CreateAxisVisual(msgs::Convert(axis1Msg.xyz()),
          axis1Msg.use_parent_model_frame(), _msg->type());
    }
  }

  if (dPtr->arrow2Visual)
    dPtr->arrow2Visual->SetVisible(false);
  if (_msg->has_axis2())
  {
    msgs::Axis axis2Msg = _msg->axis2();
    if (dPtr->arrow2Visual)
    {
      this->UpdateAxisVisual(dPtr->arrow2Visual, msgs::Convert(axis2Msg.xyz()),
          axis2Msg.use_parent_model_frame(), _msg->type());
    }
    else
    {
      dPtr->arrow2Visual = this->CreateAxisVisual(msgs::Convert(axis2Msg.xyz()),
          axis2Msg.use_parent_model_frame(), _msg->type());
    }
  }
}

/////////////////////////////////////////////////
void JointVisual::UpdateAxisVisual(ArrowVisualPtr _arrowVisual,
    const math::Vector3 &_axis, bool _useParentFrame, msgs::Joint::Type _type)
{
  JointVisualPrivate *dPtr =
      reinterpret_cast<JointVisualPrivate *>(this->dataPtr);

  // TODO generalize this and CreateAxisVisual

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
    // if set to use parent model frame
    // rotate the arrow visual relative to the model
    VisualPtr model = this->GetRootVisual();
    math::Quaternion quatFromModel =
        model->GetWorldPose().rot.GetInverse()*this->GetWorldPose().rot;
    _arrowVisual->SetRotation(quatFromModel.GetInverse() *
        _arrowVisual->GetRotation());
  }
  if (_type == msgs::Joint::REVOLUTE || _type == msgs::Joint::REVOLUTE2
      || _type == msgs::Joint::UNIVERSAL || _type == msgs::Joint::GEARBOX)
  {
    _arrowVisual->ShowRotation(true);
  }
  else
  {
    _arrowVisual->ShowRotation(false);
  }

  math::Quaternion axisWorldRotation = _arrowVisual->GetWorldPose().rot;
  math::Quaternion jointWorldRotation = this->GetWorldPose().rot;

  // Hide existing arrow head if it overlaps with the axis
  dPtr->axisVisual->ShowAxisHead(0, true);
  dPtr->axisVisual->ShowAxisHead(1, true);
  dPtr->axisVisual->ShowAxisHead(2, true);
  _arrowVisual->ShowShaft(true);
  math::Vector3 axisWorld = axisWorldRotation*math::Vector3::UnitZ;
  if (axisWorld == jointWorldRotation*math::Vector3::UnitX)
  {
    if (dPtr->axisVisual)
    {
      dPtr->axisVisual->ShowAxisHead(0, false);
      _arrowVisual->ShowShaft(false);
    }
  }
  else if (axisWorld == jointWorldRotation*math::Vector3::UnitY)
  {
    if (dPtr->axisVisual)
    {
      dPtr->axisVisual->ShowAxisHead(1, false);
      _arrowVisual->ShowShaft(false);
    }
  }
  else if (axisWorld == jointWorldRotation*math::Vector3::UnitZ)
  {
    if (dPtr->axisVisual)
    {
      dPtr->axisVisual->ShowAxisHead(2, false);
      _arrowVisual->ShowShaft(false);
    }
  }

  _arrowVisual->SetVisible(true);
}
