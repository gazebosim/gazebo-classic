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

#include "common/XMLConfig.hh"

#include "physics/Geom.hh"
#include "physics/PlaneShape.hh"

using namespace gazebo;
using namespace physics;


////////////////////////////////////////////////////////////////////////////////
/// Constructor
PlaneShape::PlaneShape(GeomPtr parent) 
  : Shape(parent)
{
  this->AddType(PLANE_SHAPE);
  this->SetName("plane_shape");

  common::Param::Begin(&this->parameters);
  this->normalP = new common::ParamT<math::Vector3>("normal",math::Vector3(0,0,1),0);
  this->normalP->Callback( &PlaneShape::SetNormal, this );
  common::Param::End();
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
PlaneShape::~PlaneShape()
{
  delete this->normalP;
}

////////////////////////////////////////////////////////////////////////////////
/// Load the plane
void PlaneShape::Load(common::XMLConfigNode *node)
{
  Shape::Load(node);
  this->normalP->Load(node->GetChild("plane"));
}

////////////////////////////////////////////////////////////////////////////////
/// Initialize the plane
void PlaneShape::Init()
{
  this->CreatePlane();
}
 
////////////////////////////////////////////////////////////////////////////////
/// Save child parameters
void PlaneShape::Save(std::string &prefix, std::ostream &stream)
{
  stream << prefix << *(this->normalP) << "\n";
}

////////////////////////////////////////////////////////////////////////////////
/// Create the plane
void PlaneShape::CreatePlane()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Set the altitude of the plane
void PlaneShape::SetAltitude(const math::Vector3 &/*_pos*/) 
{
}

////////////////////////////////////////////////////////////////////////////////
/// Set the normal
void PlaneShape::SetNormal( const math::Vector3 &norm )
{
  this->normalP->SetValue(norm);
  this->CreatePlane();
}
