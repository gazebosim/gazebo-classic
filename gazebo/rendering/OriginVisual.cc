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
/* Desc: Center of Mass Visualization Class
 * Author: Nate Koenig
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
}

/////////////////////////////////////////////////
OriginVisual::~OriginVisual()
{
}

/////////////////////////////////////////////////
void OriginVisual::Load()
{
  Visual::Load();

  OriginVisualPrivate *dPtr =
      reinterpret_cast<OriginVisualPrivate *>(this->dataPtr);

  dPtr->length = 1000;

  DynamicLines *xLine = this->CreateDynamicLine(
      rendering::RENDERING_LINE_LIST);
  xLine->setMaterial("Gazebo/Red");
  xLine->AddPoint(math::Vector3::Zero);
  xLine->AddPoint(math::Vector3::UnitX*dPtr->length);

  DynamicLines *yLine = this->CreateDynamicLine(
      rendering::RENDERING_LINE_LIST);
  yLine->setMaterial("Gazebo/Green");
  yLine->AddPoint(math::Vector3::Zero);
  yLine->AddPoint(math::Vector3::UnitY*dPtr->length);

  DynamicLines *zLine = this->CreateDynamicLine(
      rendering::RENDERING_LINE_LIST);
  zLine->setMaterial("Gazebo/Blue");
  zLine->AddPoint(math::Vector3::Zero);
  zLine->AddPoint(math::Vector3::UnitZ*dPtr->length);

  this->SetVisibilityFlags(GZ_VISIBILITY_GUI);
}

