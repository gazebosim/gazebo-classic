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
/* Desc: Joint Visualization Class
 * Author: Nate Koenig
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
}

/////////////////////////////////////////////////
void JointVisual::CreateAxis(const math::Vector3 &_axis, bool _useParentFrame,
    int _type)
{
  ArrowVisualPtr axis;
  axis.reset(new ArrowVisual(this->GetName() +
      "_axis1_AXIS", shared_from_this()));
  axis->Load();
  axis->SetMaterial("Gazebo/Yellow");

  // Get rotation to axis vector
  math::Vector3 axis1Dir = _axis;
  math::Vector3 u = axis1Dir.Normalize();
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
  if (_type == msgs::Joint::REVOLUTE)
      axis->ShowRotation();
}
