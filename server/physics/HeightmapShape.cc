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
/* Desc: Heightmap shape
 * Author: Nate Keonig, Andrew Howard
 * Date: 8 May 2003
 * CVS: $Id$
 */

#include <iostream>
#include <string.h>
#include <math.h>

#include "World.hh"
#include "Scene.hh"
#include "Image.hh"
#include "Global.hh"
#include "GazeboError.hh"
#include "Body.hh"
#include "HeightmapShape.hh"

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
HeightmapShape::HeightmapShape(Geom *parent)
    : Shape(parent)
{
  this->AddType(HEIGHTMAP_SHAPE);

  Param::Begin(&this->parameters);
  this->imageFilenameP = new ParamT<std::string>("image","",1);
  this->worldTextureP = new ParamT<std::string>("worldTexture","",0);
  this->detailTextureP = new ParamT<std::string>("detailTexture","",0);
  this->sizeP = new ParamT<Vector3>("size",Vector3(10,10,10), 0);
  this->offsetP = new ParamT<Vector3>("offset",Vector3(0,0,0), 0);
  Param::End();

  // NATY: this->ogreHeightmap = new OgreHeightmap(this->GetWorld()->GetScene());
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
HeightmapShape::~HeightmapShape()
{
  delete this->imageFilenameP;
  delete this->worldTextureP;
  delete this->detailTextureP;
  delete this->sizeP;
  delete this->offsetP;

  // NATY: delete this->ogreHeightmap;
}

//////////////////////////////////////////////////////////////////////////////
/// Update function.
void HeightmapShape::Update()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Load the heightmap
void HeightmapShape::Load(XMLConfigNode *node)
{
  this->imageFilenameP->Load(node);
  this->worldTextureP->Load(node);
  this->detailTextureP->Load(node);
  this->sizeP->Load(node);
  this->offsetP->Load(node);

  // Use the image to get the size of the heightmap
  this->img.Load( (**this->imageFilenameP) );

  // Width and height must be the same
  if (this->img.GetWidth() != this->img.GetHeight())
    gzthrow("Heightmap image must be square\n");

  this->terrainSize = (**this->sizeP);
}

////////////////////////////////////////////////////////////////////////////////
/// Save child parameters
void HeightmapShape::Save(std::string &prefix, std::ostream &stream)
{
  stream << prefix << *(this->imageFilenameP) << "\n";
  stream << prefix << *(this->worldTextureP) << "\n";
  stream << prefix << *(this->detailTextureP) << "\n";
  stream << prefix << *(this->sizeP) << "\n";
  stream << prefix << *(this->offsetP) << "\n";
}
