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

#include <ignition/math/Vector3.hh>

#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/WireBoxPrivate.hh"
#include "gazebo/rendering/WireBox.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
WireBox::WireBox(VisualPtr _parent, const ignition::math::Box &_box)
  : dataPtr(new WireBoxPrivate)
{
  this->dataPtr->parent = _parent;
  this->dataPtr->lines = new DynamicLines(RENDERING_LINE_LIST);
  this->dataPtr->lines->setMaterial("BaseWhiteNoLighting");
  this->dataPtr->parent->AttachObject(this->dataPtr->lines);
  this->dataPtr->lines->setVisibilityFlags(GZ_VISIBILITY_GUI);

  this->Init(_box);
}

/////////////////////////////////////////////////
WireBox::~WireBox()
{
  delete this->dataPtr->lines;
  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
void WireBox::Init(const ignition::math::Box &_box)
{
  ignition::math::Vector3d max = _box.Max();
  ignition::math::Vector3d min = _box.Min();

  this->dataPtr->lines->Clear();

  // line 0
  this->dataPtr->lines->AddPoint(min.X(), min.Y(), min.Z());
  this->dataPtr->lines->AddPoint(max.X(), min.Y(), min.Z());

  // line 1
  this->dataPtr->lines->AddPoint(min.X(), min.Y(), min.Z());
  this->dataPtr->lines->AddPoint(min.X(), min.Y(), max.Z());

  // line 2
  this->dataPtr->lines->AddPoint(min.X(), min.Y(), min.Z());
  this->dataPtr->lines->AddPoint(min.X(), max.Y(), min.Z());

  // line 3
  this->dataPtr->lines->AddPoint(min.X(), max.Y(), min.Z());
  this->dataPtr->lines->AddPoint(min.X(), max.Y(), max.Z());

  // line 4
  this->dataPtr->lines->AddPoint(min.X(), max.Y(), min.Z());
  this->dataPtr->lines->AddPoint(max.X(), max.Y(), min.Z());

  // line 5
  this->dataPtr->lines->AddPoint(max.X(), min.Y(), min.Z());
  this->dataPtr->lines->AddPoint(max.X(), min.Y(), max.Z());

  // line 6
  this->dataPtr->lines->AddPoint(max.X(), min.Y(), min.Z());
  this->dataPtr->lines->AddPoint(max.X(), max.Y(), min.Z());

  // line 7
  this->dataPtr->lines->AddPoint(min.X(), max.Y(), max.Z());
  this->dataPtr->lines->AddPoint(max.X(), max.Y(), max.Z());

  // line 8
  this->dataPtr->lines->AddPoint(min.X(), max.Y(), max.Z());
  this->dataPtr->lines->AddPoint(min.X(), min.Y(), max.Z());

  // line 9
  this->dataPtr->lines->AddPoint(max.X(), max.Y(), min.Z());
  this->dataPtr->lines->AddPoint(max.X(), max.Y(), max.Z());

  // line 10
  this->dataPtr->lines->AddPoint(max.X(), min.Y(), max.Z());
  this->dataPtr->lines->AddPoint(max.X(), max.Y(), max.Z());

  // line 11
  this->dataPtr->lines->AddPoint(min.X(), min.Y(), max.Z());
  this->dataPtr->lines->AddPoint(max.X(), min.Y(), max.Z());

  this->dataPtr->lines->Update();
}

/////////////////////////////////////////////////
void WireBox::SetVisible(bool _visible)
{
  this->dataPtr->lines->setVisible(_visible);
}

/////////////////////////////////////////////////
bool WireBox::GetVisible() const
{
  return this->dataPtr->lines->isVisible();
}
