/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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

#include "physics/CylinderShape.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
/// Constructor
CylinderShape::CylinderShape(CollisionPtr _parent) : Shape(_parent)
{
  this->AddType(Base::CYLINDER_SHAPE);
}

//////////////////////////////////////////////////
/// Destructor
CylinderShape::~CylinderShape()
{
}

//////////////////////////////////////////////////
/// Load the cylinder
void CylinderShape::Load(sdf::ElementPtr &_sdf)
{
  Shape::Load(_sdf);
}

//////////////////////////////////////////////////
/// Initialize the cylinder
void CylinderShape::Init()
{
  this->SetSize(this->sdf->GetValueDouble("radius"),
                 this->sdf->GetValueDouble("length"));
}


//////////////////////////////////////////////////
// Set radius
void CylinderShape::SetRadius(const double &_radius)
{
  this->sdf->GetAttribute("radius")->Set(_radius);
  this->SetSize(this->sdf->GetValueDouble("radius"),
                 this->sdf->GetValueDouble("length"));
}

//////////////////////////////////////////////////
// Set length
void CylinderShape::SetLength(const double &_length)
{
  this->sdf->GetAttribute("length")->Set(_length);
  this->SetSize(this->sdf->GetValueDouble("radius"),
                 this->sdf->GetValueDouble("length"));
}

//////////////////////////////////////////////////
/// Set the size of the cylinder
void CylinderShape::SetSize(const double &_radius, const double &_length)
{
  this->sdf->GetAttribute("radius")->Set(_radius);
  this->sdf->GetAttribute("length")->Set(_length);
}

double CylinderShape::GetRadius() const
{
  return this->sdf->GetValueDouble("radius");
}

double CylinderShape::GetLength() const
{
  return this->sdf->GetValueDouble("length");
}

void CylinderShape::FillShapeMsg(msgs::Geometry &_msg)
{
  _msg.set_type(msgs::Geometry::CYLINDER);
  _msg.mutable_cylinder()->set_radius(this->GetRadius());
  _msg.mutable_cylinder()->set_length(this->GetLength());
}

void CylinderShape::ProcessMsg(const msgs::Geometry &_msg)
{
  this->SetSize(_msg.cylinder().radius(), _msg.cylinder().length());
}


