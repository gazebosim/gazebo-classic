/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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

#include "gazebo/common/Console.hh"
#include "gazebo/physics/dart/DARTPhysics.hh"
#include "gazebo/physics/dart/DARTCollision.hh"
#include "gazebo/physics/dart/DARTSphereShape.hh"
#include "gazebo/util/system.hh"

#include "gazebo/physics/dart/DARTSphereShapePrivate.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTSphereShape::DARTSphereShape(DARTCollisionPtr _parent)
  : SphereShape(_parent),
    dataPtr(new DARTSphereShapePrivate())
{
}

//////////////////////////////////////////////////
DARTSphereShape::~DARTSphereShape()
{
  delete this->dataPtr;
}

//////////////////////////////////////////////////
void DARTSphereShape::SetRadius(double _radius)
{
  if (_radius < 0)
  {
    gzerr << "Sphere shape does not support negative radius.\n";
    return;
  }
  if (math::equal(_radius, 0.0))
  {
    // Warn user, but still create shape with very small value
    // otherwise later resize operations using setLocalScaling
    // will not be possible
    gzwarn << "Setting sphere shape's radius to zero is not supported "
           << "in DART, using 1e-4.\n";
    _radius = 1e-4;
  }

  SphereShape::SetRadius(_radius);

  DARTCollisionPtr dartCollisionParent =
      boost::dynamic_pointer_cast<DARTCollision>(this->collisionParent);

  if (dartCollisionParent->GetDARTCollisionShape() == NULL)
  {
    dart::dynamics::BodyNode *dtBodyNode =
        dartCollisionParent->GetDARTBodyNode();
    dart::dynamics::EllipsoidShape *dtEllisoidShape =
        new dart::dynamics::EllipsoidShape(Eigen::Vector3d(_radius*2.0,
                                                           _radius*2.0,
                                                           _radius*2.0));
    dtBodyNode->addCollisionShape(dtEllisoidShape);
    dartCollisionParent->SetDARTCollisionShape(dtEllisoidShape);
  }
  else
  {
    dart::dynamics::EllipsoidShape *dtEllipsoidShape =
        dynamic_cast<dart::dynamics::EllipsoidShape*>(
          dartCollisionParent->GetDARTCollisionShape());
    dtEllipsoidShape->setSize(Eigen::Vector3d(_radius*2.0,
                                              _radius*2.0,
                                              _radius*2.0));
  }
}

