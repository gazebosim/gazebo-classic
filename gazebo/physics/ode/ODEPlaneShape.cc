/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/ode/ODECollision.hh"
#include "gazebo/physics/ShapePrivate.hh"
#include "gazebo/physics/ode/ODEPlaneShape.hh"
#include "gazebo/util/system.hh"

using namespace gazebo;
using namespace physics;

/////////////////////////////////////////////////
ODEPlaneShape::ODEPlaneShape(CollisionPtr _parent)
: PlaneShape(_parent)
{
}

/////////////////////////////////////////////////
void ODEPlaneShape::CreatePlane()
{
  PlaneShape::CreatePlane();
  ODECollisionPtr oParent;
  oParent =
    std::dynamic_pointer_cast<ODECollision>(this->shapeDPtr->collisionParent);
  ignition::math::Pose3d pose = oParent->WorldPose();
  double altitude = pose.Pos().Z();
  ignition::math::Vector3d n = this->Normal();
  if (oParent->CollisionId() == NULL)
    oParent->SetCollision(dCreatePlane(oParent->SpaceId(),
          n.X(), n.Y(), n.Z(), altitude), false);
  else
    dGeomPlaneSetParams(oParent->CollisionId(),
        n.X(), n.Y(), n.Z(), altitude);
}

/////////////////////////////////////////////////
void ODEPlaneShape::SetAltitude(const ignition::math::Vector3d &_pos)
{
  PlaneShape::SetAltitude(_pos);
  ODECollisionPtr odeParent;
  odeParent =
    std::dynamic_pointer_cast<ODECollision>(this->shapeDPtr->collisionParent);

  dVector4 vec4;

  dGeomPlaneGetParams(odeParent->CollisionId(), vec4);

  // Compute "altitude": scalar product of position and normal
  vec4[3] = vec4[0] * _pos.X() + vec4[1] * _pos.Y() + vec4[2] * _pos.Z();

  dGeomPlaneSetParams(odeParent->CollisionId(), vec4[0], vec4[1],
      vec4[2], vec4[3]);
}
