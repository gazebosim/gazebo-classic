/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include "gazebo/physics/ode/ODEMesh.hh"
#include "gazebo/physics/ode/ODECollision.hh"
#include "gazebo/physics/ode/ODEPhysics.hh"
#include "gazebo/physics/ode/ODEPolyLineShape.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
ODEPolyLineShape::ODEPolyLineShape(CollisionPtr _parent)
: PolyLineShape(_parent)
{
  this->odeMesh = new ODEMesh();
}

//////////////////////////////////////////////////
ODEPolyLineShape::~ODEPolyLineShape()
{
  delete this->odeMesh;
}

//////////////////////////////////////////////////
void ODEPolyLineShape::Update()
{
  this->odeMesh->Update();
}

//////////////////////////////////////////////////
void ODEPolyLineShape::Load(sdf::ElementPtr _sdf)
{
  PolyLineShape::Load(_sdf);
}

//////////////////////////////////////////////////
void ODEPolyLineShape::Init()
{
  PolyLineShape::Init();
  if (!this->mesh)
    return;

  this->odeMesh->Init(this->mesh,
      boost::static_pointer_cast<ODECollision>(this->collisionParent),
      math::Vector3(1, 1, 1));
}
