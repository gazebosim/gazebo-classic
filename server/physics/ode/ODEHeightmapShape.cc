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
/* Desc: ODE Heightmap shape
 * Author: Nate Keonig
 * Date: 12 Nov 2009
 * SVN: $Id:$
 */

#include "Global.hh"
#include "GazeboError.hh"
#include "ODEGeom.hh"
#include "OgreHeightmap.hh"
#include "ODEHeightmapShape.hh"

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
ODEHeightmapShape::ODEHeightmapShape(Geom *parent)
    : HeightmapShape(parent)
{
}

//////////////////////////////////////////////////////////////////////////////
// Destructor
ODEHeightmapShape::~ODEHeightmapShape()
{
}

////////////////////////////////////////////////////////////////////////////////
// Create a lookup table of the terrain's height
void ODEHeightmapShape::FillHeightMap()
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
dReal ODEHeightmapShape::GetHeightCallback(void *data, int x, int y)
{
  ODEHeightmapShape *geom = (ODEHeightmapShape*)(data);

  // Return the height at a specific vertex
  return geom->heights[y * geom->odeVertSize + x];
}

////////////////////////////////////////////////////////////////////////////////
/// Load the heightmap
void ODEHeightmapShape::Load(XMLConfigNode *node)
{
  HeightmapShape::Load(node);
  ODEGeom *oParent = (ODEGeom*)(this->parent);

  // sampling size along image width and height
  this->odeVertSize = this->img.GetWidth() * 4;
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
    ODEHeightmapShape::GetHeightCallback,
    this->terrainSize.x, // in meters
    this->terrainSize.y, // in meters
    this->odeVertSize, // width sampling size
    this->odeVertSize, // depth sampling size (along height of image)
    1.0, // vertical (z-axis) scaling
    0.0, // vertical (z-axis) offset
    0.1, // vertical thickness for closing the height map mesh
    0 // wrap mode
  );

  // Step 5: Restrict the bounds of the AABB to improve efficiency
  dGeomHeightfieldDataSetBounds( this->odeData, 0, this->terrainSize.z);

  oParent->SetGeom(dCreateHeightfield( 0, this->odeData, 1), false);
  oParent->SetStatic(true);

  //Rotate so Z is up, not Y (which is the default orientation)
  Quatern quat;
  Pose3d pose = oParent->GetWorldPose();

  quat.SetFromEuler(Vector3(DTOR(90),0,0));

  pose.rot = pose.rot * quat;
  //this->body->SetPose(pose);

  dQuaternion q;
  q[0] = pose.rot.u;
  q[1] = pose.rot.x;
  q[2] = pose.rot.y;
  q[3] = pose.rot.z;

  dGeomSetQuaternion(oParent->GetGeomId(), q);
}
