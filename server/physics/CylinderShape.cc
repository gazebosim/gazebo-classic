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

////////////////////////////////////////////////////////////////////////////////
/// Constructor
CylinderShape::CylinderShape(Geom *parent) : Shape(parent)
{
  this->AddType(CYLINDER_SHAPE);

  Param::Begin(&this->parameters);
  this->sizeP = new ParamT<Vector2<double> >("size", 
      Vector2<double>(1.0,1.0), 1);
  this->sizeP->Callback( &CylinderShape::SetSize, this);
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
CylinderShape::~CylinderShape()
{
  delete this->sizeP;
}

////////////////////////////////////////////////////////////////////////////////
/// Load the cylinder
void CylinderShape::Load(XMLConfigNode *node)
{
  this->sizeP->Load(node);
  this->SetSize( this->sizeP->GetValue() );
}

////////////////////////////////////////////////////////////////////////////////
/// Save child parameters
void CylinderShape::Save(std::string &prefix, std::ostream &stream)
{
  stream << prefix << *(this->sizeP) << "\n";
}

////////////////////////////////////////////////////////////////////////////////
/// Set the size of the cylinder
void CylinderShape::SetSize( const Vector2<double> &size )
{
  this->sizeP->SetValue( size );
}


