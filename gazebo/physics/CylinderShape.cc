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

#include "gazebo/physics/CylinderShape.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
CylinderShape::CylinderShape(CollisionPtr _parent) : Shape(_parent)
{
  this->AddType(Base::CYLINDER_SHAPE);
  this->scale = math::Vector3::One;
  sdf::initFile("cylinder_shape.sdf", this->sdf);
}

//////////////////////////////////////////////////
CylinderShape::~CylinderShape()
{
}

//////////////////////////////////////////////////
void CylinderShape::Init()
{
  this->SetSize(this->sdf->Get<double>("radius"),
                 this->sdf->Get<double>("length"));
}

//////////////////////////////////////////////////
void CylinderShape::SetRadius(double _radius)
{
  this->sdf->GetElement("radius")->Set(_radius);
  if (this->sdf->HasElement("length"))
  {
    this->SetSize(_radius, this->sdf->Get<double>("length"));
  }
}

//////////////////////////////////////////////////
void CylinderShape::SetLength(double _length)
{
  this->sdf->GetElement("length")->Set(_length);
  if (this->sdf->HasElement("radius"))
  {
    this->SetSize(this->sdf->Get<double>("radius"), _length);
  }
}

//////////////////////////////////////////////////
void CylinderShape::SetSize(double _radius, double _length)
{
  this->sdf->GetElement("radius")->Set(_radius);
  this->sdf->GetElement("length")->Set(_length);
}

//////////////////////////////////////////////////
void CylinderShape::SetScale(const math::Vector3 &_scale)
{
  if (_scale.x < 0 || _scale.y < 0 || _scale.z < 0)
    return;

  if (_scale == this->scale)
    return;

  double newRadius = std::max(_scale.x, _scale.y);
  double oldRadius = std::max(this->scale.x, this->scale.y);

  this->SetRadius((newRadius/oldRadius)*this->GetRadius());
  this->SetLength((_scale.z/this->scale.z)*this->GetLength());

  this->scale = _scale;
}

/////////////////////////////////////////////////
double CylinderShape::GetRadius() const
{
  return this->sdf->Get<double>("radius");
}

/////////////////////////////////////////////////
double CylinderShape::GetLength() const
{
  return this->sdf->Get<double>("length");
}

/////////////////////////////////////////////////
void CylinderShape::FillMsg(msgs::Geometry &_msg)
{
  _msg.set_type(msgs::Geometry::CYLINDER);
  _msg.mutable_cylinder()->set_radius(this->GetRadius());
  _msg.mutable_cylinder()->set_length(this->GetLength());
}

/////////////////////////////////////////////////
void CylinderShape::ProcessMsg(const msgs::Geometry &_msg)
{
  this->SetSize(_msg.cylinder().radius(), _msg.cylinder().length());
}
