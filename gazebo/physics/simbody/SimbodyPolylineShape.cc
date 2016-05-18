/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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

#include "gazebo/common/Mesh.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/physics/simbody/SimbodyMesh.hh"
#include "gazebo/physics/simbody/SimbodyCollision.hh"
#include "gazebo/physics/simbody/SimbodyPhysics.hh"
#include "gazebo/physics/simbody/SimbodyPolylineShape.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SimbodyPolylineShape::SimbodyPolylineShape(CollisionPtr _parent)
: PolylineShape(_parent)
{
  this->simbodyMesh = new SimbodyMesh();
}

//////////////////////////////////////////////////
SimbodyPolylineShape::~SimbodyPolylineShape()
{
  delete this->simbodyMesh;
}

//////////////////////////////////////////////////
void SimbodyPolylineShape::Load(sdf::ElementPtr _sdf)
{
  PolylineShape::Load(_sdf);
}

//////////////////////////////////////////////////
void SimbodyPolylineShape::Init()
{
  PolylineShape::Init();
  if (!this->mesh)
  {
    gzerr << "Unable to create polyline in Simbody. Mesh pointer is null.\n";
    return;
  }

  gzerr << "Polyline shapes are not supported in Simbody\n";

  // Uncomment these lines when simbody supports mesh shapes.
  // this->simbodyMesh->Init(this->mesh,
  //     boost::static_pointer_cast<SimbodyCollision>(this->collisionParent),
  //     math::Vector3(1, 1, 1));
}
