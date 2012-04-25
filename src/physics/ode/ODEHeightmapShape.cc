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

  // Resize the vector to match the size of the vertices
  this->heights.resize(this->odeVertSize*this->odeVertSize);

  printf("FillHeightmap Size[%d] Scale[%f %f]\n", this->odeVertSize,
      this->odeScale.x, this->odeScale.y);

  common::Color max = this->img.GetMaxColor();
  std::cout << "Max Color[" << max << "]\n";

  common::Color pixel;
  // Iterate over all the verices
  for (y = 0; y < this->odeVertSize; y++)
  {
    for (x = 0; x < this->odeVertSize; x++)
    {
      pixel = this->img.GetPixel(x, y);
      h = pixel.R() * this->odeScale.z;

      // Store the height for future use
      this->heights[y * this->odeVertSize + x] = h;
    }
  }

  double ym = 0.0;
  //for (double ym = 0; ym < 5; ym += 1.0)
  {
    for (double xm = -64; xm < 65; xm += 1.0) 
    {
      x = (xm/this->odeScale.x) + this->img.GetWidth() * 0.5;
      y = (ym/this->odeScale.y) + this->img.GetHeight() * 0.5;
      std::cout << "Physics: Height At[" << xm << "," << ym << "]=" 
        << this->heights[y*this->odeVertSize+x] << "\n";
    }
  }
}

//////////////////////////////////////////////////
dReal ODEHeightmapShape::GetHeightCallback(void *_data, int _x, int _y)
{
  ODEHeightmapShape *collision = static_cast<ODEHeightmapShape*>(_data);

  // Return the height at a specific vertex
  return collision->heights[_y * collision->odeVertSize + _x] + 0.1;
}

//////////////////////////////////////////////////
void ODEHeightmapShape::Init()
{
  ODECollisionPtr oParent =
    boost::shared_static_cast<ODECollision>(this->collisionParent);

  math::Vector3 terrainSize = this->sdf->GetValueVector3("size");

  // sampling size along image width and height
  this->odeVertSize = this->img.GetWidth();
  this->odeScale.x = terrainSize.x / this->odeVertSize;
  this->odeScale.y = terrainSize.y / this->odeVertSize;
  if (math::equal(this->img.GetMaxColor().R(), 0))
    this->odeScale.z = terrainSize.z;
  else
    this->odeScale.z = terrainSize.z / this->img.GetMaxColor().R();

  std::cout << "Terrain Size[" << terrainSize << "]\n";
  std::cout << "Max[" << this->img.GetMaxColor().R() << "]\n";
  std::cout << "Z Scale[" << this->odeScale.z << "]\n";

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
