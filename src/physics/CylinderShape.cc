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
#include "CylinderShape.hh"

using namespace gazebo;
using namespace physics;


////////////////////////////////////////////////////////////////////////////////
/// Constructor
CylinderShape::CylinderShape(Geom *parent) : Shape(parent)
{
  this->AddType(CYLINDER_SHAPE);

  common::Param::Begin(&this->parameters);
  this->radiusP = new common::ParamT<double>("radius",1,1);
  this->radiusP->Callback( &CylinderShape::SetRadius, this);

  this->lengthP = new common::ParamT<double>("length",1,1);
  this->lengthP->Callback( &CylinderShape::SetLength, this);
  common::Param::End();
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
CylinderShape::~CylinderShape()
{
  delete this->radiusP;
  delete this->lengthP;
}

////////////////////////////////////////////////////////////////////////////////
/// Load the cylinder
void CylinderShape::Load(common::XMLConfigNode *node)
{
  this->radiusP->Load(node->GetChild("cylinder"));
  this->lengthP->Load(node->GetChild("cylinder"));

  this->SetSize( **this->radiusP, **this->lengthP );
}

////////////////////////////////////////////////////////////////////////////////
/// Save child parameters
void CylinderShape::Save(std::string &prefix, std::ostream &stream)
{
  stream << prefix << *(this->radiusP) << " " << *(this->lengthP) << "\n";
}

////////////////////////////////////////////////////////////////////////////////
// Set radius
void CylinderShape::SetRadius(const double &radius)
{
  this->SetSize(radius, **this->lengthP);
}

////////////////////////////////////////////////////////////////////////////////
// Set length
void CylinderShape::SetLength(const double &length)
{
  this->SetSize(**this->radiusP, length);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the size of the cylinder
void CylinderShape::SetSize( const double &radius, const double &length )
{
  this->radiusP->SetValue(radius);
  this->lengthP->SetValue(length);
}

