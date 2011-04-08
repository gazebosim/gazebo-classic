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

#include "common/Vector3.hh"
#include "common/XMLConfig.hh"
#include "BoxShape.hh"

using namespace gazebo;
using namespace physics;


////////////////////////////////////////////////////////////////////////////////
/// Constructor
BoxShape::BoxShape(GeomPtr parent) : Shape(parent)
{
  this->AddType(Base::BOX_SHAPE);

  common::Param::Begin(&this->parameters);
  this->sizeP = new common::ParamT<common::Vector3>("size", common::Vector3(1,1,1),1);
  this->sizeP->Callback( &BoxShape::SetSize, this );
  common::Param::End();
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
BoxShape::~BoxShape()
{
  delete this->sizeP;
} 

////////////////////////////////////////////////////////////////////////////////
/// Load the box
void BoxShape::Load(common::XMLConfigNode *node)
{
  Shape::Load(node);
  this->sizeP->Load( node->GetChild("box") );
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the box
void BoxShape::Init()
{
  this->SetSize( **this->sizeP );
}

////////////////////////////////////////////////////////////////////////////////
/// Save child parameters
void BoxShape::Save(std::string &prefix, std::ostream &stream)
{
  stream << prefix << *(this->sizeP) << "\n";
}

////////////////////////////////////////////////////////////////////////////////
/// Set the size of the box
void BoxShape::SetSize( const common::Vector3 &size )
{
  this->sizeP->SetValue( size );
}
