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


////////////////////////////////////////////////////////////////////////////////
/// Constructor
CylinderShape::CylinderShape(GeomPtr parent) : Shape(parent)
{
  this->AddType(Base::CYLINDER_SHAPE);
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
CylinderShape::~CylinderShape()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Load the cylinder
void CylinderShape::Load( sdf::ElementPtr &_sdf )
{
  Shape::Load(_sdf);
}

////////////////////////////////////////////////////////////////////////////////
/// Initialize the cylinder
void CylinderShape::Init()
{
  this->SetSize( this->sdf->GetElement("cylinder")->GetValueDouble("radius"), 
                 this->sdf->GetElement("cylinder")->GetValueDouble("length") );
}


////////////////////////////////////////////////////////////////////////////////
// Set radius
void CylinderShape::SetRadius(const double &radius)
{
  this->sdf->GetElement("cylinder")->GetAttribute("radius")->Set(radius);
  this->SetSize( this->sdf->GetElement("cylinder")->GetValueDouble("radius"), 
                 this->sdf->GetElement("cylinder")->GetValueDouble("length") );
}

////////////////////////////////////////////////////////////////////////////////
// Set length
void CylinderShape::SetLength(const double &length)
{
  this->sdf->GetElement("cylinder")->GetAttribute("length")->Set(length);
  this->SetSize( this->sdf->GetElement("cylinder")->GetValueDouble("radius"), 
                 this->sdf->GetElement("cylinder")->GetValueDouble("length") );
}

////////////////////////////////////////////////////////////////////////////////
/// Set the size of the cylinder
void CylinderShape::SetSize( const double &radius, const double &length )
{
  this->sdf->GetElement("cylinder")->GetAttribute("radius")->Set(radius);
  this->sdf->GetElement("cylinder")->GetAttribute("length")->Set(length);
  this->SetSize( this->sdf->GetElement("cylinder")->GetValueDouble("radius"), 
                 this->sdf->GetElement("cylinder")->GetValueDouble("length") );
}
