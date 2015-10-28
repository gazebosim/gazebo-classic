/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
 * Author: Nate Koenig
 * Date: 12 Nov 2009
 */

#include "gazebo/common/Exception.hh"
#include "gazebo/physics/ode/ODECollision.hh"
#include "gazebo/physics/ode/ODEHeightmapShape.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
ODEHeightmapShape::ODEHeightmapShape(CollisionPtr _parent)
    : HeightmapShape(_parent)
{
  this->flipY = false;
}

//////////////////////////////////////////////////
ODEHeightmapShape::~ODEHeightmapShape()
{
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
  HeightmapShape::Init();

  ODECollisionPtr oParent =
    boost::static_pointer_cast<ODECollision>(this->collisionParent);

  // Step 2: Create the ODE heightfield collision
  this->odeData = dGeomHeightfieldDataCreate();

  // Step 3: Setup a callback method for ODE
  dGeomHeightfieldDataBuildSingle(
      this->odeData,
      &this->heights[0],
      0,
      this->GetSize().x,  // in meters
      this->GetSize().y,  // in meters
      this->vertSize,  // width sampling size
      this->vertSize,  // depth sampling size (along height of image)
      1.0,  // vertical (z-axis) scaling
      this->GetPos().z,  // vertical (z-axis) offset
      1.0,  // vertical thickness for closing the height map mesh
      0);  // wrap mode

  // Step 4: Restrict the bounds of the AABB to improve efficiency
  dGeomHeightfieldDataSetBounds(this->odeData, 0, this->GetSize().z);

  oParent->SetCollision(dCreateHeightfield(0, this->odeData, 1), false);
  oParent->SetStatic(true);

  // Rotate so Z is up, not Y (which is the default orientation)
  math::Quaternion quat;
  math::Pose pose = oParent->GetWorldPose();

  // TODO: FIXME:  double check this, if Y is up,
  // rotating by roll of 90 deg will put Z-down.
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
