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
/* Desc: Sphere shape
 * Author: Nate Keonig
 * Date: 14 Oct 2009
 */
#include "common/XMLConfig.hh"
#include "physics/SphereShape.hh"

using namespace gazebo;
using namespace physics;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
SphereShape::SphereShape(GeomPtr parent) 
  : Shape(parent)
{
  this->AddType(Base::SPHERE_SHAPE);

  common::Param::Begin(&this->parameters);
  this->radiusP = new common::ParamT<double>("radius",1.0,0);
  this->radiusP->Callback( &SphereShape::SetSize, this );
  common::Param::End();
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
SphereShape::~SphereShape()
{
  delete this->radiusP;
}

////////////////////////////////////////////////////////////////////////////////
/// Load the sphere
void SphereShape::Load(common::XMLConfigNode *node)
{
  Shape::Load(node);
  this->radiusP->Load( node->GetChild("sphere") );
}

////////////////////////////////////////////////////////////////////////////////
/// Initialize the sphere
void SphereShape::Init()
{
  this->SetSize( **this->radiusP );
}
 
////////////////////////////////////////////////////////////////////////////////
/// Save shape parameters
void SphereShape::Save(std::string &prefix, std::ostream &stream)
{
  stream << prefix << *(this->radiusP) << "\n";
}

////////////////////////////////////////////////////////////////////////////////
/// Set the size
void SphereShape::SetSize(const double &radius)
{
  this->radiusP->SetValue( radius );
}


