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
 */

#include "common/Exception.hh"
#include "physics/ode/ODECollision.hh"
#include "physics/ode/ODEHeightmapShape.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
ODEHeightmapShape::ODEHeightmapShape(CollisionPtr _parent)
    : HeightmapShape(_parent)
{
}

//////////////////////////////////////////////////
ODEHeightmapShape::~ODEHeightmapShape()
{
}

//////////////////////////////////////////////////
void ODEHeightmapShape::FillHeightMap()
{
  unsigned int x, y;
  float h = 0;
  float h1 = 0;
  float h2 = 0;

  // Resize the vector to match the size of the vertices
  this->heights.resize(this->odeVertSize*this->odeVertSize);

  common::Color pixel;

  double yf, xf, dy, dx;
  int y1, y2, x1, x2;
  double px1, px2, px3, px4;

  int imgHeight = this->img.GetHeight();
  int imgWidth = this->img.GetWidth();
  // Bytes per row
  unsigned int pitch = this->img.GetPitch();
  // Bytes per pixel
  unsigned int bpp = pitch / imgWidth;

  unsigned char *data = NULL;
  unsigned int count;
  this->img.GetData(&data, count);

  // Iterate over all the verices
  for (y = 0; y < this->odeVertSize; y++)
  {
    yf = y / static_cast<double>(this->subSampling);
    y1 = floor(yf);
    y2 = ceil(yf);
    if (y2 >= imgHeight)
      y2 = y1;
    dy = yf - y1;

    for (x = 0; x < this->odeVertSize; x++)
    {
      xf = x / static_cast<double>(this->subSampling);
      x1 = floor(xf);
      x2 = ceil(xf);
      if (x2 >= imgWidth)
        x2 = x1;

      dx = xf - x1;

      px1 = static_cast<int>(data[y1 * pitch + x1 * bpp]) / 255.0;
      px2 = static_cast<int>(data[y1 * pitch + x2 * bpp]) / 255.0;
      h1 = (px1 - ((px1 - px2) * dx));

      px3 = static_cast<int>(data[y2 * pitch + x1 * bpp]) / 255.0;
      px4 = static_cast<int>(data[y2 * pitch + x2 * bpp]) / 255.0;
      h2 = (px3 - ((px3 - px4) * dx));

      h = (h1 - ((h1 - h2) * dy)) * this->odeScale.z;

      // Store the height for future use
      this->heights[y * this->odeVertSize + x] = h;
    }
  }

  delete [] data;
}

//////////////////////////////////////////////////
dReal ODEHeightmapShape::GetHeightCallback(void *_data, int _x, int _y)
{
  // Return the height at a specific vertex
  return static_cast<ODEHeightmapShape*>(_data)->GetHeight(_x, _y);
}

//////////////////////////////////////////////////
void ODEHeightmapShape::Init()
{
  ODECollisionPtr oParent =
    boost::shared_static_cast<ODECollision>(this->collisionParent);

  math::Vector3 terrainSize = this->sdf->GetValueVector3("size");

  this->subSampling = 4;

  // sampling size along image width and height
  this->odeVertSize = this->img.GetWidth() * this->subSampling;
  this->odeScale.x = terrainSize.x / this->odeVertSize;
  this->odeScale.y = terrainSize.y / this->odeVertSize;
  if (math::equal(this->img.GetMaxColor().R(), 0))
    this->odeScale.z = terrainSize.z;
  else
    this->odeScale.z = terrainSize.z / this->img.GetMaxColor().R();

  // Step 1: Construct the heightmap lookup table, using the ogre ray scene
  // query functionality
  this->FillHeightMap();

  // Step 2: Create the ODE heightfield collision
  this->odeData = dGeomHeightfieldDataCreate();

  // Step 3: Setup a callback method for ODE
  dGeomHeightfieldDataBuildCallback(
      this->odeData,
      this,
      ODEHeightmapShape::GetHeightCallback,
      terrainSize.x,  // in meters
      terrainSize.y,  // in meters
      this->odeVertSize,  // width sampling size
      this->odeVertSize,  // depth sampling size (along height of image)
      1.0,  // vertical (z-axis) scaling
      0.0,  // vertical (z-axis) offset
      1.0,  // vertical thickness for closing the height map mesh
      0);  // wrap mode

  // Step 4: Restrict the bounds of the AABB to improve efficiency
  dGeomHeightfieldDataSetBounds(this->odeData, 0, terrainSize.z);

  oParent->SetCollision(dCreateHeightfield(0, this->odeData, 1), false);
  oParent->SetStatic(true);

  // Rotate so Z is up, not Y (which is the default orientation)
  math::Quaternion quat;
  math::Pose pose = oParent->GetWorldPose();

  quat.SetFromEuler(math::Vector3(GZ_DTOR(90), 0, 0));

  pose.rot = pose.rot * quat;
  // this->body->SetPose(pose);

  dQuaternion q;
  q[0] = pose.rot.w;
  q[1] = pose.rot.x;
  q[2] = pose.rot.y;
  q[3] = pose.rot.z;

  dGeomSetQuaternion(oParent->GetCollisionId(), q);
}

/////////////////////////////////////////////////
virtual float ODEHeightmapShape::GetHeight(int _x, int _y)
{
  return this->heights[_y * this->odeVertSize + _x];
}
