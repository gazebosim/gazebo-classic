/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#include "gazebo/rendering/AxisVisual.hh"
#include "gazebo/rendering/LinkOriginVisualPrivate.hh"
#include "gazebo/rendering/LinkOriginVisual.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
LinkOriginVisual::LinkOriginVisual(const std::string &_name, VisualPtr _parent)
  : AxisVisual(*new LinkOriginVisualPrivate, _name, _parent)
{
  LinkOriginVisualPrivate *dPtr =
      reinterpret_cast<LinkOriginVisualPrivate *>(this->dataPtr);

  dPtr->type = VT_PHYSICS;
}

/////////////////////////////////////////////////
LinkOriginVisual::~LinkOriginVisual()
{
}

/////////////////////////////////////////////////
void LinkOriginVisual::Load(ConstLinkPtr &_msg)
{
  AxisVisual::Load(_msg);

  this->ShowAxisHead(0, false);
  this->ShowAxisHead(1, false);
  this->ShowAxisHead(2, false);
}
