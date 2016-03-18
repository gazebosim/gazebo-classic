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
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/ShapePrivate.hh"
#include "gazebo/physics/SphereShape.hh"
#include "gazebo/physics/ode/ODEPhysics.hh"
#include "gazebo/physics/ode/ODECollision.hh"
#include "gazebo/physics/ode/ODESphereShape.hh"

using namespace gazebo;
using namespace physics;

/////////////////////////////////////////////////
ODESphereShape::ODESphereShape(ODECollisionPtr _parent)
: SphereShape(_parent)
{
}

/////////////////////////////////////////////////
void ODESphereShape::SetRadius(const double _radius)
{
  SphereShape::SetRadius(_radius);
  ODECollisionPtr oParent;
  oParent =
    std::dynamic_pointer_cast<ODECollision>(this->shapeDPtr->collisionParent);

  // Create the sphere geometry
  if (oParent->CollisionId() == NULL)
    oParent->SetCollision(dCreateSphere(0, _radius), true);
  else
    dGeomSphereSetRadius(oParent->CollisionId(), _radius);
}
