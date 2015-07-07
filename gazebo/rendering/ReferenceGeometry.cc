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
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/ReferenceGeometryPrivate.hh"
#include "gazebo/rendering/ReferenceGeometry.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
ReferenceGeometry::ReferenceGeometry(const std::string &_name, VisualPtr _vis)
  : Visual(*new ReferenceGeometryPrivate, _name, _vis, false)
{
  ReferenceGeometryPrivate *dPtr =
      reinterpret_cast<ReferenceGeometryPrivate *>(this->dataPtr);

  dPtr->type = VT_GUI;
  dPtr->referenceGeometryType = RGT_NONE;
}

/////////////////////////////////////////////////
ReferenceGeometry::~ReferenceGeometry()
{
  ReferenceGeometryPrivate *dPtr =
      reinterpret_cast<ReferenceGeometryPrivate *>(this->dataPtr);

  if (dPtr->axisLine)
  {
    this->DeleteDynamicLine(dPtr->axisLine);
    dPtr->axisLine = NULL;
  }
}

/////////////////////////////////////////////////
void ReferenceGeometry::Load()
{
  ReferenceGeometryPrivate *dPtr =
      reinterpret_cast<ReferenceGeometryPrivate *>(this->dataPtr);

  Visual::Load();

  if (dPtr->referenceGeometryType == RGT_AXIS)
    this->CreateAxis();


  this->SetVisibilityFlags(GZ_VISIBILITY_GUI);
}

/////////////////////////////////////////////////
void ReferenceGeometry::SetReferenceGeometryType(ReferenceGeometryType _type)
{
  ReferenceGeometryPrivate *dPtr =
      reinterpret_cast<ReferenceGeometryPrivate *>(this->dataPtr);

  dPtr->referenceGeometryType = _type;

  if (dPtr->referenceGeometryType == RGT_AXIS)
  {
    if (!dPtr->axisVisual)
      dPtr->axisVisual = this->CreateAxis();
    if (!dPtr->axisLine)
    {
      dPtr->axisLine = this->CreateDynamicLine();
      dPtr->axisLine->AddPoint(0, 0, -0.5);
      dPtr->axisLine->AddPoint(0, 0, 0.5);
    }
    this->SetTransparency(0.5);
  }
}

/////////////////////////////////////////////////
ReferenceGeometry::ReferenceGeometryType
    ReferenceGeometry::GetReferenceGeometryType() const
{
  ReferenceGeometryPrivate *dPtr =
      reinterpret_cast<ReferenceGeometryPrivate *>(this->dataPtr);

  return dPtr->referenceGeometryType;
}


/////////////////////////////////////////////////
VisualPtr ReferenceGeometry::CreateAxis()
{
  VisualPtr cylinderVis;
  cylinderVis.reset(new Visual("reference_geom_axis", shared_from_this()));
  cylinderVis->Load();
  cylinderVis->AttachMesh("unit_cylinder");
  cylinderVis->SetScale(math::Vector3(0.1, 0.1, 1.0));
  cylinderVis->SetMaterial("Gazebo/DottedLine");
//  cylinderVis->SetVisibilityFlags(GZ_VISIBILITY_SELECTABLE);

  return cylinderVis;
}
