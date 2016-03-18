/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include "gazebo/physics/simbody/SimbodySphereShape.hh"

using namespace gazebo;
using namespace physics;

/////////////////////////////////////////////////
SimbodySphereShape::SimbodySphereShape(CollisionPtr _parent)
: SphereShape(_parent)
{
}

/////////////////////////////////////////////////
SimbodySphereShape::~SimbodySphereShape()
{
}

/////////////////////////////////////////////////
void SimbodySphereShape::SetRadius(double _radius)
{
  if (_radius < 0)
  {
    gzerr << "Sphere shape does not support negative radius\n";
    return;
  }

  if (ignition::math::equal(_radius, 0.0))
  {
    // Warn user, but still create shape with very small value
    // otherwise later resize operations using setLocalScaling
    // will not be possible
    gzwarn << "Setting sphere shape's radius to zero \n";
    _radius = 1e-4;
  }

  SphereShape::SetRadius(_radius);
  SimbodyCollisionPtr bParent;
  bParent = std::dynamic_pointer_cast<SimbodyCollision>(
      this->shapeDPtr->collisionParent);

  // set collision shape
}
