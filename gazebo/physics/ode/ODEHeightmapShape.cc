/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
      // in meters
      this->Size().X(),
      // in meters
      this->Size().Y(),
      // width sampling size
      this->vertSize,
      // depth sampling size (along height of image)
      this->vertSize,
      // vertical (z-axis) scaling
      1.0,
      // vertical (z-axis) offset
      this->Pos().Z(),
      // vertical thickness for closing the height map mesh
      1.0,
      // wrap mode
      0);

  // Step 4: Restrict the bounds of the AABB to improve efficiency
  dGeomHeightfieldDataSetBounds(this->odeData, this->GetMinHeight(),
      this->GetMaxHeight());

  oParent->SetCollision(dCreateHeightfield(0, this->odeData, 1), false);
  oParent->SetStatic(true);

  // Rotate so Z is up, not Y (which is the default orientation)
  // TODO: FIXME:  double check this, if Y is up,
  // rotating by roll of 90 deg will put Z-down.
  ignition::math::Quaterniond quat(IGN_DTOR(90), 0, 0);

  ignition::math::Pose3d pose = oParent->WorldPose();

  pose.Rot() = pose.Rot() * quat;
  // this->body->SetPose(pose);

  dQuaternion q;
  q[0] = pose.Rot().W();
  q[1] = pose.Rot().X();
  q[2] = pose.Rot().Y();
  q[3] = pose.Rot().Z();

  dGeomSetQuaternion(oParent->GetCollisionId(), q);
}
