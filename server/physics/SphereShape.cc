/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: Sphere shape
 * Author: Nate Keonig
 * Date: 14 Oct 2009
 * SVN: $Id:$
 */

#include "SphereShape.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
SphereShape::SphereShape(Geom *parent) : Shape(parent)
{
  this->type.push_back("sphere");

  Param::Begin(&this->parameters);
  this->radiusP = new ParamT<double>("size",1.0,0);
  this->radiusP->Callback( &SphereShape::SetSize, this );
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
SphereShape::~SphereShape()
{
  delete this->radiusP;
}

////////////////////////////////////////////////////////////////////////////////
/// Load the sphere
void SphereShape::Load(XMLConfigNode *node)
{
  this->radiusP->Load(node);
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


