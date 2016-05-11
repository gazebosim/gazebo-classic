/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include "gazebo/physics/ShapePrivate.hh"
#include "gazebo/physics/CylinderShape.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
CylinderShape::CylinderShape(CollisionPtr _parent) : Shape(_parent)
{
  this->AddType(Base::CYLINDER_SHAPE);
  this->shapeDPtr->scale = ignition::math::Vector3d::One;
  sdf::initFile("cylinder_shape.sdf", this->shapeDPtr->sdf);
}

//////////////////////////////////////////////////
CylinderShape::~CylinderShape()
{
}

//////////////////////////////////////////////////
void CylinderShape::Init()
{
  this->SetSize(this->shapeDPtr->sdf->Get<double>("radius"),
                 this->shapeDPtr->sdf->Get<double>("length"));
}

//////////////////////////////////////////////////
void CylinderShape::SetRadius(double _radius)
{
  this->shapeDPtr->sdf->GetElement("radius")->Set(_radius);
  if (this->shapeDPtr->sdf->HasElement("length"))
  {
    this->SetSize(_radius, this->shapeDPtr->sdf->Get<double>("length"));
  }
}

//////////////////////////////////////////////////
void CylinderShape::SetLength(double _length)
{
  this->shapeDPtr->sdf->GetElement("length")->Set(_length);
  if (this->shapeDPtr->sdf->HasElement("radius"))
  {
    this->SetSize(this->shapeDPtr->sdf->Get<double>("radius"), _length);
  }
}

//////////////////////////////////////////////////
void CylinderShape::SetSize(double _radius, double _length)
{
  this->shapeDPtr->sdf->GetElement("radius")->Set(_radius);
  this->shapeDPtr->sdf->GetElement("length")->Set(_length);
}

//////////////////////////////////////////////////
void CylinderShape::SetScale(const math::Vector3 &_scale)
{
  this->SetScale(_scale.Ign());
}

//////////////////////////////////////////////////
void CylinderShape::SetScale(const ignition::math::Vector3d &_scale)
{
  if (_scale.Min() < 0)
    return;

  if (_scale == this->shapeDPtr->scale)
    return;

  double newRadius = std::max(_scale.X(), _scale.Y());
  double oldRadius = std::max(this->shapeDPtr->scale.X(),
      this->shapeDPtr->scale.Y());

  this->SetRadius((newRadius/oldRadius)*this->Radius());
  this->SetLength((_scale.Z()/this->shapeDPtr->scale.Z())*this->Length());

  this->shapeDPtr->scale = _scale;
}

/////////////////////////////////////////////////
double CylinderShape::GetRadius() const
{
  return this->Radius();
}

/////////////////////////////////////////////////
double CylinderShape::Radius() const
{
  return this->shapeDPtr->sdf->Get<double>("radius");
}

/////////////////////////////////////////////////
double CylinderShape::GetLength() const
{
  return this->Length();
}

/////////////////////////////////////////////////
double CylinderShape::Length() const
{
  return this->shapeDPtr->sdf->Get<double>("length");
}

/////////////////////////////////////////////////
void CylinderShape::FillMsg(msgs::Geometry &_msg)
{
  _msg.set_type(msgs::Geometry::CYLINDER);
  _msg.mutable_cylinder()->set_radius(this->Radius());
  _msg.mutable_cylinder()->set_length(this->Length());
}

/////////////////////////////////////////////////
void CylinderShape::ProcessMsg(const msgs::Geometry &_msg)
{
  this->SetSize(_msg.cylinder().radius(), _msg.cylinder().length());
}

/////////////////////////////////////////////////
double CylinderShape::ComputeVolume() const
{
  return IGN_CYLINDER_VOLUME(this->Radius(), this->Length());
}
