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
/* Desc: Joint Visualization Class
 * Author: Nate Koenig
 */

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/AxisVisual.hh"
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

  // joint axis is in the model frame so set up the scene node to be
  // the same orientation as the model then apply rotations later.
  VisualPtr model = this->GetRootVisual();
  if (model)
  {
    math::Quaternion quat = model->GetRotation();
    this->GetSceneNode()->_setDerivedOrientation(Conversions::Convert(quat));
  }

  dPtr->axisVisual.reset(
      new AxisVisual(this->GetName() + "_AXIS", shared_from_this()));
  dPtr->axisVisual->Load();

  this->SetPosition(msgs::Convert(_msg->pose().position()));
  this->SetRotation(this->GetRotation() *
      msgs::Convert(_msg->pose().orientation()));

  if (math::equal(_msg->axis1().xyz().x(), 1.0))
    dPtr->axisVisual->ShowRotation(0);

  if (math::equal(_msg->axis1().xyz().y(), 1.0))
    dPtr->axisVisual->ShowRotation(1);

  if (math::equal(_msg->axis1().xyz().z(), 1.0))
    dPtr->axisVisual->ShowRotation(2);
}
