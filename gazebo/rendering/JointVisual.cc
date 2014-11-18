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

  // create an arrow visual for each axis in the joint message
  if (_msg->has_axis1())
  {
    msgs::Axis axis1Msg = _msg->axis1();
    this->CreateAxis(msgs::Convert(axis1Msg.xyz()),
        axis1Msg.use_parent_model_frame(), _msg->type());
  }
  if (_msg->has_axis2())
  {
    msgs::Axis axis2Msg = _msg->axis2();
    this->CreateAxis(msgs::Convert(axis2Msg.xyz()),
        axis2Msg.use_parent_model_frame(), _msg->type());
  }

  this->GetSceneNode()->setInheritScale(false);
  this->SetVisibilityFlags(GZ_VISIBILITY_GUI);
}

/////////////////////////////////////////////////
void JointVisual::CreateAxis(const math::Vector3 &_axis, bool _useParentFrame,
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
      axis->ShowRotation();

  math::Quaternion axisWorldRotation = axis->GetWorldPose().rot;
  math::Quaternion jointWorldRotation = this->GetWorldPose().rot;

  // hide the existing axis visual if it overlaps with the one we are creating
  math::Vector3 axisWorld = axisWorldRotation*math::Vector3::UnitZ;
  if (axisWorld == jointWorldRotation*math::Vector3::UnitX)
    dPtr->axisVisual->SetAxisVisible(0, false);
  else if (axisWorld == jointWorldRotation*math::Vector3::UnitY)
    dPtr->axisVisual->SetAxisVisible(1, false);
  else if (axisWorld == jointWorldRotation*math::Vector3::UnitZ)
    dPtr->axisVisual->SetAxisVisible(2, false);

  dPtr->arrowVisuals.push_back(axis);
}

/////////////////////////////////////////////////
void JointVisual::ShowAxis(unsigned int _index, bool _show)
{
  JointVisualPrivate *dPtr =
      reinterpret_cast<JointVisualPrivate *>(this->dataPtr);

  if (_index >= dPtr->arrowVisuals.size())
    return;

  if (_show)
    this->AttachVisual(dPtr->arrowVisuals[_index]);
  else
    this->DetachVisual(dPtr->arrowVisuals[_index]);
}
