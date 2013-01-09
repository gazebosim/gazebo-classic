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

#include "gazebo/common/Exception.hh"
#include "gazebo/math/Quaternion.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/gui/model_editor/BuildingMaker.hh"
#include "gazebo/gui/model_editor/ModelManip.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ModelManip::ModelManip()
{
  this->parent = NULL;
}

/////////////////////////////////////////////////
ModelManip::~ModelManip()
{
  this->DetachFromParent();
}

/////////////////////////////////////////////////
void ModelManip::SetName(const std::string &_name)
{
  this->name = _name;
}

/////////////////////////////////////////////////
void ModelManip::SetVisual(const rendering::VisualPtr &_visual)
{
  this->visual = _visual;
}

/////////////////////////////////////////////////
std::string ModelManip::GetName() const
{
  return this->name;
}

/////////////////////////////////////////////////
rendering::VisualPtr ModelManip::GetVisual() const
{
  return this->visual;
}

/////////////////////////////////////////////////
void ModelManip::SetMaker(BuildingMaker *_maker)
{
  this->maker = _maker;
}

/////////////////////////////////////////////////
ModelManip *ModelManip::GetParent() const
{
  return this->parent;
}

/////////////////////////////////////////////////
void ModelManip::OnSizeChanged(double _width, double _depth, double _height)
{
  this->size = BuildingMaker::ConvertSize(_width, _depth, _height);
  double dScaleZ = this->visual->GetScale().z - this->size.z;
  this->visual->SetScale(this->size);
  math::Vector3 originalPos = this->visual->GetPosition();
  math::Vector3 newPos = originalPos
      - math::Vector3(0, 0, dScaleZ/2.0);
  this->visual->SetPosition(newPos);
}

/////////////////////////////////////////////////
void ModelManip::AttachObject(ModelManip *_object)
{
  if (!_object->IsAttached())
  {
    _object->SetAttachedTo(this);
    this->attachedObjects.push_back(_object);
  }
}

/////////////////////////////////////////////////
void ModelManip::DetachObject(ModelManip *_object)
{
  if (_object)
  {
    std::vector<ModelManip *> ::iterator it = std::remove(
        this->attachedObjects.begin(), this->attachedObjects.end(), _object);
    if (it != this->attachedObjects.end())
    {
      _object->DetachFromParent();
      this->attachedObjects.erase(it, this->attachedObjects.end());
    }
  }
}

/////////////////////////////////////////////////
void ModelManip::DetachFromParent()
{
  if (this->parent)
  {
    ModelManip *tmp = this->parent;
    this->parent = NULL;
    tmp->DetachObject(this);
  }
}

/////////////////////////////////////////////////
void ModelManip::SetAttachedTo(ModelManip *_parent)
{
  if (this->IsAttached())
  {
    gzerr << this->name << " is already attached to a parent \n";
    return;
  }
  this->parent = _parent;
}

/////////////////////////////////////////////////
ModelManip *ModelManip::GetAttachedObject(unsigned int _index) const
{
  if (_index >= this->attachedObjects.size())
    gzthrow("Index too large");

  return this->attachedObjects[_index];
}

/////////////////////////////////////////////////
unsigned int ModelManip::GetAttachedObjectCount() const
{
  return this->attachedObjects.size();
}

/////////////////////////////////////////////////
bool ModelManip::IsAttached() const
{
  return (this->parent != NULL);
}

/////////////////////////////////////////////////
void ModelManip::OnPoseChanged(double _x, double _y, double _z,
    double _roll, double _pitch, double _yaw)
{
  this->SetPose(_x, _y, _z, _roll, _pitch, _yaw);
}

/////////////////////////////////////////////////
void ModelManip::OnPoseOriginTransformed(double _x, double _y, double _z,
    double _roll, double _pitch, double _yaw)
{
  // Handle translations, currently used by polylines
  math::Pose trans = BuildingMaker::ConvertPose(_x, -_y, _z, _roll, _pitch,
      _yaw);

  math::Pose oldPose = this->visual->GetParent()->GetWorldPose();

  this->visual->GetParent()->SetWorldPose(oldPose + trans);
}

/////////////////////////////////////////////////
void ModelManip::OnPositionChanged(double _x, double _y, double _z)
{
  double scaledX = BuildingMaker::Convert(_x);
  double scaledY = BuildingMaker::Convert(-_y);
  double scaledZ = BuildingMaker::Convert(_z);

  this->visual->GetParent()->SetWorldPosition(math::Vector3(
      scaledX, scaledY, scaledZ));
}

/////////////////////////////////////////////////
void ModelManip::OnWidthChanged(double _width)
{
  double scaledWidth = BuildingMaker::Convert(_width);
  this->size = this->visual->GetScale();
  this->size.x = scaledWidth;
  this->visual->SetScale(this->size);
}

/////////////////////////////////////////////////
void ModelManip::OnDepthChanged(double _depth)
{
  double scaledDepth = BuildingMaker::Convert(_depth);
  this->size = this->visual->GetScale();
  this->size.y = scaledDepth;
  this->visual->SetScale(this->size);
}

/////////////////////////////////////////////////
void ModelManip::OnHeightChanged(double _height)
{
  double scaledHeight = BuildingMaker::Convert(_height);
  this->size = this->visual->GetScale();
  this->size.z = scaledHeight;
  math::Vector3 dScale = this->visual->GetScale() - this->size;
  math::Vector3 originalPos = this->visual->GetPosition();
  this->visual->SetScale(this->size);

  math::Vector3 newPos = originalPos
      - math::Vector3(0, 0, dScale.z/2.0);

  this->visual->SetPosition(newPos);
}

/////////////////////////////////////////////////
void ModelManip::OnPosXChanged(double _posX)
{
  math::Pose visualPose = this->visual->GetParent()->GetWorldPose();
  double scaledX = BuildingMaker::Convert(_posX);
  visualPose.pos.x = scaledX;
  this->visual->GetParent()->SetWorldPosition(visualPose.pos);
}

/////////////////////////////////////////////////
void ModelManip::OnPosYChanged(double _posY)
{
  math::Pose visualPose = this->visual->GetParent()->GetWorldPose();
  double scaledY = BuildingMaker::Convert(_posY);
  visualPose.pos.y = -scaledY;
  this->visual->GetParent()->SetWorldPosition(visualPose.pos);
}

/////////////////////////////////////////////////
void ModelManip::OnPosZChanged(double _posZ)
{
  math::Pose visualPose = this->visual->GetParent()->GetWorldPose();
  double scaledZ = BuildingMaker::Convert(_posZ);
  visualPose.pos.z = scaledZ;
  this->visual->GetParent()->SetWorldPosition(visualPose.pos);
}

/////////////////////////////////////////////////
void ModelManip::OnYawChanged(double _yaw)
{
  double newYaw = BuildingMaker::ConvertAngle(_yaw);
  math::Vector3 angles = this->visual->GetRotation().GetAsEuler();
  angles.z = -newYaw;
  this->visual->GetParent()->SetRotation(angles);
}

/////////////////////////////////////////////////
void ModelManip::OnRotationChanged(double _roll, double _pitch, double _yaw)
{
  this->SetRotation(_roll, _pitch, _yaw);
}

/////////////////////////////////////////////////
void ModelManip::OnItemDeleted()
{
  this->maker->RemovePart(this->name);
}

/////////////////////////////////////////////////
void ModelManip::SetPose(double _x, double _y, double _z,
    double _roll, double _pitch, double _yaw)
{
  this->SetPosition(_x, _y, _z);
  this->SetRotation(_roll, _pitch, _yaw);
}

/////////////////////////////////////////////////
void ModelManip::SetPosition(double _x, double _y, double _z)
{
  double scaledX = BuildingMaker::Convert(_x);
  double scaledY = BuildingMaker::Convert(-_y);
  double scaledZ = BuildingMaker::Convert(_z);
  this->visual->GetParent()->SetWorldPosition(math::Vector3(scaledX, scaledY,
      scaledZ));
}

/////////////////////////////////////////////////
void ModelManip::SetRotation(double _roll, double _pitch, double _yaw)
{
  double rollRad = BuildingMaker::ConvertAngle(_roll);
  double pitchRad = BuildingMaker::ConvertAngle(_pitch);
  double yawRad = BuildingMaker::ConvertAngle(_yaw);

  this->visual->GetParent()->SetRotation(
      math::Quaternion(rollRad, pitchRad, -yawRad));
}

/////////////////////////////////////////////////
void ModelManip::SetSize(double _width, double _depth, double _height)
{
  this->size = BuildingMaker::ConvertSize(_width, _depth, _height);

  math::Vector3 dScale = this->visual->GetScale() - this->size;

  math::Vector3 originalPos = this->visual->GetPosition();
  this->visual->SetPosition(math::Vector3(0, 0, 0));
  this->visual->SetScale(this->size);

  // adjust position due to difference in pivot points
  math::Vector3 newPos = originalPos
      - math::Vector3(dScale.x/2.0, dScale.y/2.0, dScale.z/2.0);

  this->visual->SetPosition(newPos);
}
