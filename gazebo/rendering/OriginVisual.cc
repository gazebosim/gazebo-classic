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

#include "gazebo/math/Vector3.hh"

#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/OriginVisualPrivate.hh"
#include "gazebo/rendering/OriginVisual.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
OriginVisual::OriginVisual(const std::string &_name, VisualPtr _vis)
  : Visual(*new OriginVisualPrivate, _name, _vis, false)
{
  OriginVisualPrivate *dPtr =
      reinterpret_cast<OriginVisualPrivate *>(this->dataPtr);
  dPtr->type = VT_GUI;
}

/////////////////////////////////////////////////
OriginVisual::~OriginVisual()
{
  OriginVisualPrivate *dPtr =
      reinterpret_cast<OriginVisualPrivate *>(this->dataPtr);

  this->DeleteDynamicLine(dPtr->xLine);
  this->DeleteDynamicLine(dPtr->yLine);
  this->DeleteDynamicLine(dPtr->zLine);
}

/////////////////////////////////////////////////
void OriginVisual::Load()
{
  Visual::Load();

  OriginVisualPrivate *dPtr =
      reinterpret_cast<OriginVisualPrivate *>(this->dataPtr);

  dPtr->length = 1000;

  dPtr->xLine = this->CreateDynamicLine(
      rendering::RENDERING_LINE_LIST);
  dPtr->xLine->setMaterial("Gazebo/Red");
  dPtr->xLine->AddPoint(math::Vector3::Zero);
  dPtr->xLine->AddPoint(math::Vector3::UnitX*dPtr->length);

  dPtr->yLine = this->CreateDynamicLine(
      rendering::RENDERING_LINE_LIST);
  dPtr->yLine->setMaterial("Gazebo/Green");
  dPtr->yLine->AddPoint(math::Vector3::Zero);
  dPtr->yLine->AddPoint(math::Vector3::UnitY*dPtr->length);

  dPtr->zLine = this->CreateDynamicLine(
      rendering::RENDERING_LINE_LIST);
  dPtr->zLine->setMaterial("Gazebo/Blue");
  dPtr->zLine->AddPoint(math::Vector3::Zero);
  dPtr->zLine->AddPoint(math::Vector3::UnitZ*dPtr->length);

  this->SetVisibilityFlags(GZ_VISIBILITY_GUI);
}
