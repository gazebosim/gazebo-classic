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

#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/WireBox.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
WireBox::WireBox(VisualPtr _parent, const math::Box &_box)
{
  this->parent = _parent;
  this->lines = new DynamicLines(RENDERING_LINE_LIST);
  this->lines->setMaterial("BaseWhiteNoLighting");
  this->parent->AttachObject(this->lines);
  this->lines->setVisibilityFlags(GZ_VISIBILITY_GUI);

  this->Init(_box);
}

/////////////////////////////////////////////////
WireBox::~WireBox()
{
  delete this->lines;
}

/////////////////////////////////////////////////
void WireBox::Init(const math::Box &_box)
{
  math::Vector3 max = _box.max;
  math::Vector3 min = _box.min;

  this->lines->Clear();

  // line 0
  this->lines->AddPoint(min.x, min.y, min.z);
  this->lines->AddPoint(max.x, min.y, min.z);

  // line 1
  this->lines->AddPoint(min.x, min.y, min.z);
  this->lines->AddPoint(min.x, min.y, max.z);

  // line 2
  this->lines->AddPoint(min.x, min.y, min.z);
  this->lines->AddPoint(min.x, max.y, min.z);

  // line 3
  this->lines->AddPoint(min.x, max.y, min.z);
  this->lines->AddPoint(min.x, max.y, max.z);

  // line 4
  this->lines->AddPoint(min.x, max.y, min.z);
  this->lines->AddPoint(max.x, max.y, min.z);

  // line 5
  this->lines->AddPoint(max.x, min.y, min.z);
  this->lines->AddPoint(max.x, min.y, max.z);

  // line 6
  this->lines->AddPoint(max.x, min.y, min.z);
  this->lines->AddPoint(max.x, max.y, min.z);

  // line 7
  this->lines->AddPoint(min.x, max.y, max.z);
  this->lines->AddPoint(max.x, max.y, max.z);

  // line 8
  this->lines->AddPoint(min.x, max.y, max.z);
  this->lines->AddPoint(min.x, min.y, max.z);

  // line 9
  this->lines->AddPoint(max.x, max.y, min.z);
  this->lines->AddPoint(max.x, max.y, max.z);

  // line 10
  this->lines->AddPoint(max.x, min.y, max.z);
  this->lines->AddPoint(max.x, max.y, max.z);

  // line 11
  this->lines->AddPoint(min.x, min.y, max.z);
  this->lines->AddPoint(max.x, min.y, max.z);

  this->lines->Update();
}

/////////////////////////////////////////////////
void WireBox::SetVisible(bool _visible)
{
  this->lines->setVisible(_visible);
}
