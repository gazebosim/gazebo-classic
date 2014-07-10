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
#include <ignition/math/Quaternion.hh>
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
  double dScaleZ = this->visual->GetScale().z() - this->size.z();
  this->visual->SetScale(this->size);
  ignition::math::Vector3d originalPos = this->visual->GetPosition();
  ignition::math::Vector3d newPos = originalPos
      - ignition::math::Vector3d(0, 0, dScaleZ/2.0);
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
  ignition::math::Pose3d trans = BuildingMaker::ConvertPose(_x, -_y, _z, _roll, _pitch,
      _yaw);

  ignition::math::Pose3d oldPose = this->visual->GetParent()->GetWorldPose();

  this->visual->GetParent()->SetWorldPose(oldPose + trans);
}

/////////////////////////////////////////////////
void BuildingModelManip::OnPositionChanged(double _x, double _y, double _z)
{
  double scaledX = BuildingMaker::Convert(_x);
  double scaledY = BuildingMaker::Convert(-_y);
  double scaledZ = BuildingMaker::Convert(_z);

  this->visual->GetParent()->SetWorldPosition(ignition::math::Vector3d(
      scaledX, scaledY, scaledZ));
}

/////////////////////////////////////////////////
void BuildingModelManip::OnWidthChanged(double _width)
{
  double scaledWidth = BuildingMaker::Convert(_width);
  this->size = this->visual->GetScale();
  this->size.x() = scaledWidth;
  this->visual->SetScale(this->size);
}

/////////////////////////////////////////////////
void BuildingModelManip::OnDepthChanged(double _depth)
{
  double scaledDepth = BuildingMaker::Convert(_depth);
  this->size = this->visual->GetScale();
  this->size.y() = scaledDepth;
  this->visual->SetScale(this->size);
}

/////////////////////////////////////////////////
void BuildingModelManip::OnHeightChanged(double _height)
{
  double scaledHeight = BuildingMaker::Convert(_height);
  this->size = this->visual->GetScale();
  this->size.z() = scaledHeight;
  ignition::math::Vector3d dScale = this->visual->GetScale() - this->size;
  ignition::math::Vector3d originalPos = this->visual->GetPosition();
  this->visual->SetScale(this->size);

  ignition::math::Vector3d newPos = originalPos
      - ignition::math::Vector3d(0, 0, dScale.z()/2.0);

  this->visual->SetPosition(newPos);
}

/////////////////////////////////////////////////
void BuildingModelManip::OnPosXChanged(double _posX)
{
  ignition::math::Pose3d visualPose = this->visual->GetParent()->GetWorldPose();
  double scaledX = BuildingMaker::Convert(_posX);
  visualPose.Pos().x() = scaledX;
  this->visual->GetParent()->SetWorldPosition(visualPose.Pos());
}

/////////////////////////////////////////////////
void BuildingModelManip::OnPosYChanged(double _posY)
{
  ignition::math::Pose3d visualPose = this->visual->GetParent()->GetWorldPose();
  double scaledY = BuildingMaker::Convert(_posY);
  visualPose.Pos().y() = -scaledY;
  this->visual->GetParent()->SetWorldPosition(visualPose.Pos());
}

/////////////////////////////////////////////////
void BuildingModelManip::OnPosZChanged(double _posZ)
{
  ignition::math::Pose3d visualPose = this->visual->GetParent()->GetWorldPose();
  double scaledZ = BuildingMaker::Convert(_posZ);
  visualPose.Pos().z() = scaledZ;
  this->visual->GetParent()->SetWorldPosition(visualPose.Pos());
}

/////////////////////////////////////////////////
void BuildingModelManip::OnYawChanged(double _yaw)
{
  double newYaw = BuildingMaker::ConvertAngle(_yaw);
  ignition::math::Vector3d angles = this->visual->Rotation().Euler();
  angles.z() = -newYaw;
  this->visual->GetParent()->SetRotation(angles);
}

/////////////////////////////////////////////////
void BuildingModelManip::OnRotationChanged(double _roll, double _pitch,
    double _yaw)
{
  this->SetRotation(_roll, _pitch, _yaw);
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
  this->visual->GetParent()->SetWorldPosition(ignition::math::Vector3d(scaledX, scaledY,
      scaledZ));
}

/////////////////////////////////////////////////
void BuildingModelManip::SetRotation(double _roll, double _pitch, double _yaw)
{
  double rollRad = BuildingMaker::ConvertAngle(_roll);
  double pitchRad = BuildingMaker::ConvertAngle(_pitch);
  double yawRad = BuildingMaker::ConvertAngle(_yaw);

  this->visual->GetParent()->SetRotation(
      ignition::math::Quaterniond(rollRad, pitchRad, -yawRad));
}

/////////////////////////////////////////////////
void BuildingModelManip::SetSize(double _width, double _depth, double _height)
{
  this->size = BuildingMaker::ConvertSize(_width, _depth, _height);

  ignition::math::Vector3d dScale = this->visual->GetScale() - this->size;

  ignition::math::Vector3d originalPos = this->visual->GetPosition();
  this->visual->SetPosition(ignition::math::Vector3d(0, 0, 0));
  this->visual->SetScale(this->size);

  // adjust position due to difference in pivot points
  ignition::math::Vector3d newPos = originalPos
      - ignition::math::Vector3d(dScale.x()/2.0, dScale.y()/2.0, dScale.z()/2.0);

  this->visual->SetPosition(newPos);
}
