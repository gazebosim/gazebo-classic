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
/* Desc: Heightmap geometry
 * Author: Nate Keonig, Andrew Howard
 * Date: 8 May 2003
 * CVS: $Id$
 */

#include <ode/ode.h>
#include <iostream>
#include <string.h>
#include <math.h>

#include "Image.hh"
#include "Global.hh"
#include "OgreHeightmap.hh"
#include "GazeboError.hh"
#include "Body.hh"
#include "HeightmapGeom.hh"

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
HeightmapGeom::HeightmapGeom(Body *body)
    : Geom(body)
{
  Param::Begin(&this->parameters);
  this->imageFilenameP = new ParamT<std::string>("image","",1);
  this->worldTextureP = new ParamT<std::string>("worldTexture","",0);
  this->detailTextureP = new ParamT<std::string>("detailTexture","",0);
  this->sizeP = new ParamT<Vector3>("size",Vector3(10,10,10), 0);
  this->offsetP = new ParamT<Vector3>("offset",Vector3(0,0,0), 0);
  Param::End();

  this->ogreHeightmap = new OgreHeightmap();
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
HeightmapGeom::~HeightmapGeom()
{
  delete this->imageFilenameP;
  delete this->worldTextureP;
  delete this->detailTextureP;
  delete this->sizeP;
  delete this->offsetP;

  delete this->ogreHeightmap;
}

//////////////////////////////////////////////////////////////////////////////
/// Update function.
void HeightmapGeom::UpdateChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Create a lookup table of the terrain's height
void HeightmapGeom::FillHeightMap()
{
  unsigned int x,y;
  float h;

  // Resize the vector to match the size of the vertices
  this->heights.resize(this->odeVertSize*this->odeVertSize);

  // Iterate over all the verices
  for (y=0; y<this->odeVertSize; y++)
  {
    for (x=0; x<this->odeVertSize; x++)
    {
      // Find the height at a vertex
      h = this->ogreHeightmap->GetHeightAt(Vector2<float>(x*this->odeScale.x, y*this->odeScale.y));

      // Store the height for future use
      this->heights[y*this->odeVertSize + x] = h;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// Called by ODE to get the height at a vertex
dReal HeightmapGeom::GetHeightCallback(void *data, int x, int y)
{
  HeightmapGeom *geom = (HeightmapGeom*)(data);

  // Return the height at a specific vertex
  return geom->heights[y * geom->odeVertSize + x];
}

////////////////////////////////////////////////////////////////////////////////
/// Load the heightmap
void HeightmapGeom::LoadChild(XMLConfigNode *node)
{
  Image tmpImage;

  this->imageFilenameP->Load(node);
  this->worldTextureP->Load(node);
  this->detailTextureP->Load(node);
  this->sizeP->Load(node);
  this->offsetP->Load(node);

  // Use the image to get the size of the heightmap
  tmpImage.Load( (**this->imageFilenameP) );

  // Width and height must be the same
  if (tmpImage.GetWidth() != tmpImage.GetHeight())
  {
    gzthrow("Heightmap image must be square\n");
  }

  this->terrainSize = (**this->sizeP);
  this->odeVertSize = tmpImage.GetWidth() * 4;
  this->odeScale = this->terrainSize / this->odeVertSize;


  /*std::ostringstream stream;
  std::cout << "ODE Scale[" << this->odeScale << "]\n";
  std::cout << "Terrain Image[" << this->imageFilenameP->GetValue() << "] Size[" << this->terrainSize << "]\n";
  printf("Terrain Size[%f %f %f]\n", this->terrainSize.x, this->terrainSize.y, this->terrainSize.z);
  */

  // Step 1: Create the Ogre height map: Performs a ray scene query
  this->ogreHeightmap->Load( (**this->imageFilenameP), (**this->worldTextureP),
      (**this->detailTextureP), this->terrainSize );

  // Step 2: Construct the heightmap lookup table, using the ogre ray scene
  // query functionality
  this->FillHeightMap();

  // Step 3: Create the ODE heightfield geom
  this->odeData = dGeomHeightfieldDataCreate();

  // Step 4: Setup a callback method for ODE
  dGeomHeightfieldDataBuildCallback(
    this->odeData,
    this,
    HeightmapGeom::GetHeightCallback,
    this->terrainSize.x,
    this->terrainSize.y,
    this->odeVertSize,
    this->odeVertSize,
    1.0, // scale
    0.0, // vertical offset
    0.0, // vertical thickness
    0 // wrap mode
  );

  // Step 5: Restrict the bounds of the AABB to improve efficiency
  dGeomHeightfieldDataSetBounds( this->odeData, 0, this->terrainSize.z);

  this->geomId = dCreateHeightfield( this->spaceId, this->odeData, 1);

  this->SetGeom(this->geomId, false);

  this->SetStatic(true);

  //Rotate so Z is up, not Y (which is the default orientation)
  Quatern quat;
  Pose3d pose = this->GetPose();

  quat.SetFromEuler(Vector3(DTOR(90),0,0));

  pose.rot = pose.rot * quat;
  this->body->SetPose(pose);
}

////////////////////////////////////////////////////////////////////////////////
/// Save child parameters
void HeightmapGeom::SaveChild(std::string &prefix, std::ostream &stream)
{
  stream << prefix << *(this->imageFilenameP) << "\n";
  stream << prefix << *(this->worldTextureP) << "\n";
  stream << prefix << *(this->detailTextureP) << "\n";
  stream << prefix << *(this->sizeP) << "\n";
  stream << prefix << *(this->offsetP) << "\n";
}
