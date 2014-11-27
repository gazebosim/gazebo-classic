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

#include "gazebo/rendering/Visual.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/math/Quaternion.hh"
#include "gazebo/gui/building/BuildingMaker.hh"
#include "gazebo/gui/building/BuildingModelManip.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
BuildingModelManip::BuildingModelManip()
{
  this->parent = NULL;
}

/////////////////////////////////////////////////
BuildingModelManip::~BuildingModelManip()
{
  this->DetachFromParent();
}

/////////////////////////////////////////////////
void BuildingModelManip::SetName(const std::string &_name)
{
  this->name = _name;
}

/////////////////////////////////////////////////
void BuildingModelManip::SetVisual(const rendering::VisualPtr &_visual)
{
  this->visual = _visual;
}

/////////////////////////////////////////////////
std::string BuildingModelManip::GetName() const
{
  return this->name;
}

/////////////////////////////////////////////////
rendering::VisualPtr BuildingModelManip::GetVisual() const
{
  return this->visual;
}

/////////////////////////////////////////////////
common::Color BuildingModelManip::GetColor() const
{
  return this->color;
}

/////////////////////////////////////////////////
void BuildingModelManip::SetMaker(BuildingMaker *_maker)
{
  this->maker = _maker;
}

/////////////////////////////////////////////////
BuildingModelManip *BuildingModelManip::GetParent() const
{
  return this->parent;
}

/////////////////////////////////////////////////
void BuildingModelManip::OnSizeChanged(double _width, double _depth,
    double _height)
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
void BuildingModelManip::AttachManip(BuildingModelManip *_manip)
{
  if (!_manip->IsAttached())
  {
    _manip->SetAttachedTo(this);
    this->attachedManips.push_back(_manip);
  }
}

/////////////////////////////////////////////////
void BuildingModelManip::DetachManip(BuildingModelManip *_manip)
{
  if (_manip)
  {
    std::vector<BuildingModelManip *> ::iterator it = std::remove(
        this->attachedManips.begin(), this->attachedManips.end(), _manip);
    if (it != this->attachedManips.end())
    {
      _manip->DetachFromParent();
      this->attachedManips.erase(it, this->attachedManips.end());
    }
  }
}

/////////////////////////////////////////////////
void BuildingModelManip::DetachFromParent()
{
  if (this->parent)
  {
    BuildingModelManip *tmp = this->parent;
    this->parent = NULL;
    tmp->DetachManip(this);
  }
}

/////////////////////////////////////////////////
void BuildingModelManip::SetAttachedTo(BuildingModelManip *_parent)
{
  if (this->IsAttached())
  {
    gzerr << this->name << " is already attached to a parent \n";
    return;
  }
  this->parent = _parent;
}

/////////////////////////////////////////////////
BuildingModelManip *BuildingModelManip::GetAttachedManip(
    unsigned int _index) const
{
  if (_index >= this->attachedManips.size())
    gzthrow("Index too large");

  return this->attachedManips[_index];
}

/////////////////////////////////////////////////
unsigned int BuildingModelManip::GetAttachedManipCount() const
{
  return this->attachedManips.size();
}

/////////////////////////////////////////////////
bool BuildingModelManip::IsAttached() const
{
  return (this->parent != NULL);
}

/////////////////////////////////////////////////
void BuildingModelManip::OnPoseChanged(double _x, double _y, double _z,
    double _roll, double _pitch, double _yaw)
{
  this->SetPose(_x, _y, _z, _roll, _pitch, _yaw);
}

/////////////////////////////////////////////////
void BuildingModelManip::OnPoseOriginTransformed(double _x, double _y,
    double _z, double _roll, double _pitch, double _yaw)
{
  // Handle translations, currently used by polylines
  math::Pose trans = BuildingMaker::ConvertPose(_x, -_y, _z, _roll, _pitch,
      _yaw);

  math::Pose oldPose = this->visual->GetParent()->GetWorldPose();

  this->visual->GetParent()->SetWorldPose(oldPose + trans);
}

/////////////////////////////////////////////////
void BuildingModelManip::OnPositionChanged(double _x, double _y, double _z)
{
  double scaledX = BuildingMaker::Convert(_x);
  double scaledY = BuildingMaker::Convert(-_y);
  double scaledZ = BuildingMaker::Convert(_z);

  this->visual->GetParent()->SetWorldPosition(math::Vector3(
      scaledX, scaledY, scaledZ));
}

/////////////////////////////////////////////////
void BuildingModelManip::OnWidthChanged(double _width)
{
  double scaledWidth = BuildingMaker::Convert(_width);
  this->size = this->visual->GetScale();
  this->size.x = scaledWidth;
  this->visual->SetScale(this->size);
}

/////////////////////////////////////////////////
void BuildingModelManip::OnDepthChanged(double _depth)
{
  double scaledDepth = BuildingMaker::Convert(_depth);
  this->size = this->visual->GetScale();
  this->size.y = scaledDepth;
  this->visual->SetScale(this->size);
}

/////////////////////////////////////////////////
void BuildingModelManip::OnHeightChanged(double _height)
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
void BuildingModelManip::OnPosXChanged(double _posX)
{
  math::Pose visualPose = this->visual->GetParent()->GetWorldPose();
  double scaledX = BuildingMaker::Convert(_posX);
  visualPose.pos.x = scaledX;
  this->visual->GetParent()->SetWorldPosition(visualPose.pos);
}

/////////////////////////////////////////////////
void BuildingModelManip::OnPosYChanged(double _posY)
{
  math::Pose visualPose = this->visual->GetParent()->GetWorldPose();
  double scaledY = BuildingMaker::Convert(_posY);
  visualPose.pos.y = -scaledY;
  this->visual->GetParent()->SetWorldPosition(visualPose.pos);
}

/////////////////////////////////////////////////
void BuildingModelManip::OnPosZChanged(double _posZ)
{
  math::Pose visualPose = this->visual->GetParent()->GetWorldPose();
  double scaledZ = BuildingMaker::Convert(_posZ);
  visualPose.pos.z = scaledZ;
  this->visual->GetParent()->SetWorldPosition(visualPose.pos);
}

/////////////////////////////////////////////////
void BuildingModelManip::OnYawChanged(double _yaw)
{
  double newYaw = BuildingMaker::ConvertAngle(_yaw);
  math::Vector3 angles = this->visual->GetRotation().GetAsEuler();
  angles.z = -newYaw;
  this->visual->GetParent()->SetRotation(angles);
}

/////////////////////////////////////////////////
void BuildingModelManip::OnRotationChanged(double _roll, double _pitch,
    double _yaw)
{
  this->SetRotation(_roll, _pitch, _yaw);
}

/////////////////////////////////////////////////
void BuildingModelManip::OnColorChanged(QColor _color)
{
  this->SetColor(_color);
}

/////////////////////////////////////////////////
void BuildingModelManip::OnTransparencyChanged(float _transparency)
{
  this->SetTransparency(_transparency);
}

/////////////////////////////////////////////////
void BuildingModelManip::OnDeleted()
{
  this->maker->RemovePart(this->name);
}

/////////////////////////////////////////////////
void BuildingModelManip::SetPose(double _x, double _y, double _z,
    double _roll, double _pitch, double _yaw)
{
  this->SetPosition(_x, _y, _z);
  this->SetRotation(_roll, _pitch, _yaw);
}

/////////////////////////////////////////////////
void BuildingModelManip::SetPosition(double _x, double _y, double _z)
{
  double scaledX = BuildingMaker::Convert(_x);
  double scaledY = BuildingMaker::Convert(-_y);
  double scaledZ = BuildingMaker::Convert(_z);
  this->visual->GetParent()->SetWorldPosition(math::Vector3(scaledX, scaledY,
      scaledZ));
}

/////////////////////////////////////////////////
void BuildingModelManip::SetRotation(double _roll, double _pitch, double _yaw)
{
  double rollRad = BuildingMaker::ConvertAngle(_roll);
  double pitchRad = BuildingMaker::ConvertAngle(_pitch);
  double yawRad = BuildingMaker::ConvertAngle(_yaw);

  this->visual->GetParent()->SetRotation(
      math::Quaternion(rollRad, pitchRad, -yawRad));
}

/////////////////////////////////////////////////
void BuildingModelManip::SetSize(double _width, double _depth, double _height)
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

/////////////////////////////////////////////////
void BuildingModelManip::SetColor(QColor _color)
{
  common::Color newColor(_color.red(), _color.green(), _color.blue());
  this->color = newColor;
  this->visual->GetParent()->SetAmbient(this->color);
}

/////////////////////////////////////////////////
void BuildingModelManip::SetTransparency(float _transparency)
{
  this->visual->GetParent()->SetTransparency(_transparency);
}
