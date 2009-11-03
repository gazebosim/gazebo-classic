/* *  Gazebo - Outdoor Multi-Robot Simulator
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

#include "Geom.hh"
#include "OgreCreator.hh"
#include "PlaneShape.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
PlaneShape::PlaneShape(Geom *parent) : Shape(parent)
{
  this->type = Shape::PLANE;

  Param::Begin(&this->parameters);
  this->normalP = new ParamT<Vector3>("normal",Vector3(0,0,1),0);
  this->normalP->Callback( &PlaneShape::SetNormal, this );

  this->sizeP = new ParamT<Vector2<double> >("size",
      Vector2<double>(1000, 1000), 0);
  this->sizeP->Callback( &PlaneShape::SetSize, this );

  this->segmentsP = new ParamT<Vector2<double> >("segments",
      Vector2<double>(10, 10), 0);
  this->segmentsP->Callback( &PlaneShape::SetSegments, this );

  this->uvTileP = new ParamT<Vector2<double> >("uvTile",
      Vector2<double>(1, 1), 0);
  this->uvTileP->Callback( &PlaneShape::SetUVTile, this );

  this->materialP = new ParamT<std::string>("material","",1);
  this->materialP->Callback( &PlaneShape::SetMaterial, this );

  this->castShadowsP = new ParamT<bool>("castShadows", false, 0);
  this->castShadowsP->Callback( &PlaneShape::SetCastShadows, this );
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
PlaneShape::~PlaneShape()
{
  delete this->normalP;
  delete this->sizeP;
  delete this->segmentsP;
  delete this->uvTileP;
  delete this->materialP;
  delete this->castShadowsP;
}

////////////////////////////////////////////////////////////////////////////////
/// Load the plane
void PlaneShape::Load(XMLConfigNode *node)
{
  Vector3 perp;

  this->normalP->Load(node);
  this->sizeP->Load(node);
  this->segmentsP->Load(node);
  this->uvTileP->Load(node);
  this->materialP->Load(node);
  this->castShadowsP->Load(node);
  this->CreatePlane();
}

////////////////////////////////////////////////////////////////////////////////
/// Save child parameters
void PlaneShape::Save(std::string &prefix, std::ostream &stream)
{
  stream << prefix << *(this->normalP) << "\n";
  stream << prefix << *(this->sizeP) << "\n";
  stream << prefix << *(this->segmentsP) << "\n";
  stream << prefix << *(this->uvTileP) << "\n";
  stream << prefix << *(this->materialP) << "\n";
  stream << prefix << *(this->castShadowsP) << "\n";
}

////////////////////////////////////////////////////////////////////////////////
/// Create the plane
void PlaneShape::CreatePlane()
{
  this->meshName = OgreCreator::CreatePlane(**(this->normalP), 
      **(this->sizeP), **(this->segmentsP), **(this->uvTileP), 
      **(this->materialP), **(this->castShadowsP), 
      this->parent->GetVisualNode(), this->meshName);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the altitude of the plane
void PlaneShape::SetAltitude(const Vector3 &pos) 
{
}

////////////////////////////////////////////////////////////////////////////////
/// Set the normal
void PlaneShape::SetNormal( const Vector3 &norm )
{
  OgreCreator::RemoveMesh(this->meshName);

  this->normalP->SetValue( norm );
  this->CreatePlane();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the size
void PlaneShape::SetSize( const Vector2<double> &size )
{
  OgreCreator::RemoveMesh(this->meshName);
  this->sizeP->SetValue( size );
  this->CreatePlane();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the number of segments
void PlaneShape::SetSegments(const Vector2<double> &seg)
{
  OgreCreator::RemoveMesh(this->meshName);
  this->segmentsP->SetValue( seg );
  this->CreatePlane();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the uvtile
void PlaneShape::SetUVTile(const Vector2<double> &uv)
{
  OgreCreator::RemoveMesh(this->meshName);
  this->uvTileP->SetValue( uv );
  this->CreatePlane();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the material
void PlaneShape::SetMaterial(const std::string &mat)
{
  OgreCreator::RemoveMesh(this->meshName);
  this->materialP->SetValue( mat );
  this->CreatePlane();
}

////////////////////////////////////////////////////////////////////////////////
/// Set cast shadows
void PlaneShape::SetCastShadows(const bool &cast)
{
  OgreCreator::RemoveMesh(this->meshName);
  this->castShadowsP->SetValue( cast );
  this->CreatePlane();
}

