/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include "gazebo/common/MeshManager.hh"

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/ArrowVisual.hh"
#include "gazebo/rendering/AxisVisualPrivate.hh"
#include "gazebo/rendering/AxisVisual.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
AxisVisual::AxisVisual(const std::string &_name, VisualPtr _parent)
  : Visual(*new AxisVisualPrivate, _name, _parent, false)
{
  AxisVisualPrivate *dPtr =
      reinterpret_cast<AxisVisualPrivate *>(this->dataPtr);

  dPtr->type = VT_GUI;
}

//////////////////////////////////////////////////
AxisVisual::AxisVisual(VisualPrivate &_dataPtr, const std::string &_name,
    VisualPtr _parent)
    : Visual(_dataPtr, _name, _parent, false)
{
  AxisVisualPrivate *dPtr =
      reinterpret_cast<AxisVisualPrivate *>(this->dataPtr);

  dPtr->type = VT_GUI;
}

/////////////////////////////////////////////////
AxisVisual::~AxisVisual()
{
  AxisVisualPrivate *dPtr =
      reinterpret_cast<AxisVisualPrivate *>(this->dataPtr);

  dPtr->xAxis.reset();
  dPtr->yAxis.reset();
  dPtr->zAxis.reset();
}

/////////////////////////////////////////////////
void AxisVisual::Load()
{
  AxisVisualPrivate *dPtr =
      reinterpret_cast<AxisVisualPrivate *>(this->dataPtr);

  Visual::Load();

  dPtr->xAxis.reset(new ArrowVisual(this->GetName() +
      "_X_AXIS", shared_from_this()));
  dPtr->xAxis->Load();
  dPtr->xAxis->SetMaterial("Gazebo/RedTransparent");

  dPtr->yAxis.reset(new ArrowVisual(this->GetName() +
      "_Y_AXIS", shared_from_this()));
  dPtr->yAxis->Load();
  dPtr->yAxis->SetMaterial("Gazebo/GreenTransparent");

  dPtr->zAxis.reset(new ArrowVisual(this->GetName() +
      "_Z_AXIS", shared_from_this()));
  dPtr->zAxis->Load();
  dPtr->zAxis->SetMaterial("Gazebo/BlueTransparent");

  dPtr->xAxis->SetRotation(
      ignition::math::Quaterniond(ignition::math::Vector3d(0, 1, 0),
        IGN_DTOR(90)));

  dPtr->yAxis->SetRotation(
      ignition::math::Quaterniond(ignition::math::Vector3d(1, 0, 0),
        IGN_DTOR(-90)));

  this->SetVisibilityFlags(GZ_VISIBILITY_GUI);
}

/////////////////////////////////////////////////
void AxisVisual::ScaleXAxis(const math::Vector3 &_scale)
{
  this->ScaleXAxis(_scale.Ign());
}

/////////////////////////////////////////////////
void AxisVisual::ScaleXAxis(const ignition::math::Vector3d &_scale)
{

  AxisVisualPrivate *dPtr =
      reinterpret_cast<AxisVisualPrivate *>(this->dataPtr);

  dPtr->xAxis->SetScale(_scale);
}

/////////////////////////////////////////////////
void AxisVisual::ScaleYAxis(const math::Vector3 &_scale)
{
  this->ScaleYAxis(_scale.Ign());
}

/////////////////////////////////////////////////
void AxisVisual::ScaleYAxis(const ignition::math::Vector3d &_scale)
{
  AxisVisualPrivate *dPtr =
      reinterpret_cast<AxisVisualPrivate *>(this->dataPtr);

  dPtr->yAxis->SetScale(_scale);
}

/////////////////////////////////////////////////
void AxisVisual::ScaleZAxis(const math::Vector3 &_scale)
{
  this->ScaleZAxis(_scale.Ign());
}

/////////////////////////////////////////////////
void AxisVisual::ScaleZAxis(const ignition::math::Vector3d &_scale)
{
  AxisVisualPrivate *dPtr =
      reinterpret_cast<AxisVisualPrivate *>(this->dataPtr);

  dPtr->zAxis->SetScale(_scale);
}

/////////////////////////////////////////////////
void AxisVisual::SetAxisMaterial(unsigned int _axis,
                                 const std::string &_material)
{
  AxisVisualPrivate *dPtr =
      reinterpret_cast<AxisVisualPrivate *>(this->dataPtr);

  switch (_axis)
  {
    case 0:
      dPtr->xAxis->SetMaterial(_material);
      break;
    case 1:
      dPtr->yAxis->SetMaterial(_material);
      break;
    case 2:
      dPtr->zAxis->SetMaterial(_material);
      break;
    default:
      gzerr << "Invalid axis index[" << _axis << "]\n";
      break;
  };
}

/////////////////////////////////////////////////
void AxisVisual::ShowAxisRotation(unsigned int _axis, bool _show)
{
  AxisVisualPrivate *dPtr =
      reinterpret_cast<AxisVisualPrivate *>(this->dataPtr);

  switch (_axis)
  {
    case 0:
      dPtr->xAxis->ShowRotation(_show);
      break;
    case 1:
      dPtr->yAxis->ShowRotation(_show);
      break;
    case 2:
      dPtr->zAxis->ShowRotation(_show);
      break;
    default:
      gzerr << "Invalid axis index[" << _axis << "]\n";
      break;
  };
}


/////////////////////////////////////////////////
void AxisVisual::ShowAxisShaft(unsigned int _axis, bool _show)
{
  AxisVisualPrivate *dPtr =
      reinterpret_cast<AxisVisualPrivate *>(this->dataPtr);

  switch (_axis)
  {
    case 0:
      dPtr->xAxis->ShowShaft(_show);
      break;
    case 1:
      dPtr->yAxis->ShowShaft(_show);
      break;
    case 2:
      dPtr->zAxis->ShowShaft(_show);
      break;
    default:
      gzerr << "Invalid axis index[" << _axis << "]\n";
      break;
  };
}

/////////////////////////////////////////////////
void AxisVisual::ShowAxisHead(unsigned int _axis, bool _show)
{
  AxisVisualPrivate *dPtr =
      reinterpret_cast<AxisVisualPrivate *>(this->dataPtr);

  switch (_axis)
  {
    case 0:
      dPtr->xAxis->ShowHead(_show);
      break;
    case 1:
      dPtr->yAxis->ShowHead(_show);
      break;
    case 2:
      dPtr->zAxis->ShowHead(_show);
      break;
    default:
      gzerr << "Invalid axis index[" << _axis << "]\n";
      break;
  };
}

/////////////////////////////////////////////////
void AxisVisual::SetAxisVisible(unsigned int _axis, bool _visible)
{
  AxisVisualPrivate *dPtr =
      reinterpret_cast<AxisVisualPrivate *>(this->dataPtr);

  VisualPtr axis;
  switch (_axis)
  {
    case 0:
      axis = dPtr->xAxis;
      break;
    case 1:
      axis = dPtr->yAxis;
      break;
    case 2:
      axis = dPtr->zAxis;
      break;
    default:
      gzerr << "Invalid axis index[" << _axis << "]" << std::endl;
      return;
  };

  axis->SetVisible(_visible);
}
