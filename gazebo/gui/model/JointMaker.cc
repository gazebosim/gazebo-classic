/*
 * Copyright 2012 Open Source Robotics Foundation
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
#include "gazebo/gui/model/JointMaker.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
JointMaker::JointMaker(const std::string &_name,
    rendering::VisualPtr _vis)
: rendering::Visual(_name, _vis)
{
  this->jointLine =
      this->CreateDynamicLine(rendering::RENDERING_LINE_LIST);
  this->jointLine->AddPoint(math::Vector3(0, 0, 0));
  this->jointLine->AddPoint(math::Vector3(0, 0, 0.01));
}

/////////////////////////////////////////////////
JointMaker::~JointMaker()
{
  delete this->jointLine;
}

/////////////////////////////////////////////////
void JointMaker::SetParent(rendering::VisualPtr _parent,
    math::Vector3 _offset)
{
  if (!_parent)
  {
    gzerr << "Parent is NULL" << std::endl;
    return;
  }

  if (_parent != this->parent)
  {
    this->parent->DetachVisual(this->GetName());
    this->parent = _parent;
    this->parent->AttachVisual(shared_from_this());
  }

  this->jointLine->SetPoint(0, _offset);
}

/////////////////////////////////////////////////
void JointMaker::SetChild(rendering::VisualPtr _child,
    math::Vector3 _offset)
{
  if (!_child)
  {
    gzerr << "Child is NULL" << std::endl;
    return;
  }
  math::Vector3 parentPos = math::Vector3::Zero;
  if (this->parent)
    parentPos = this->parent->GetWorldPose().pos;
  this->jointLine->SetPoint(0,
      _child->GetWorldPose().pos - parentPos + _offset);
}
