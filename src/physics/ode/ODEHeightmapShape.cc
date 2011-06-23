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
/* Desc: ODE Heightmap shape
 * Author: Nate Keonig
 * Date: 12 Nov 2009
 * SVN: $Id:$
 */

#include "common/Global.hh"
#include "common/Exception.hh"
#include "physics/ode/ODEGeom.hh"
#include "physics/ode/ODEHeightmapShape.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////////////////////////////////
// Constructor
ODEHeightmapShape::ODEHeightmapShape(GeomPtr parent)
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
  float h = 0;

  // Resize the vector to match the size of the vertices
  this->heights.resize(this->odeVertSize*this->odeVertSize);

  // Iterate over all the verices
  for (y=0; y<this->odeVertSize; y++)
  {
    for (x=0; x<this->odeVertSize; x++)
    {
      // Find the height at a vertex
      //NATY: h = this->ogreHeightmap->GetHeightAt(math::Vector2<float>(x*this->odeScale.x, y*this->odeScale.y));

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
void ODEHeightmapShape::Init()
{
  ODEGeomPtr oParent = boost::shared_static_cast<ODEGeom>(this->geomParent);

  // sampling size along image width and height
  this->odeVertSize = this->img.GetWidth() * 4;
  this->odeScale = this->terrainSize / this->odeVertSize;

  // Step 1: Create the Ogre height map: Performs a ray scene query
  //NATY this->ogreHeightmap->Load( (**this->imageFilenameP), (**this->worldTextureP), (**this->detailTextureP), this->terrainSize );

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
  math::Quaternion quat;
  math::Pose pose = oParent->GetWorldPose();

  quat.SetFromEuler(math::Vector3(DTOR(90),0,0));

  pose.rot = pose.rot * quat;
  //this->body->SetPose(pose);

  dQuaternion q;
  q[0] = pose.rot.w;
  q[1] = pose.rot.x;
  q[2] = pose.rot.y;
  q[3] = pose.rot.z;

  dGeomSetQuaternion(oParent->GetGeomId(), q);
}
