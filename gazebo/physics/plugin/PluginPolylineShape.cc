/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#include "gazebo/physics/plugin/PluginMesh.hh"
#include "gazebo/physics/plugin/PluginCollision.hh"
#include "gazebo/physics/plugin/PluginPhysics.hh"
#include "gazebo/physics/plugin/PluginPolylineShape.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
PluginPolylineShape::PluginPolylineShape(CollisionPtr _parent)
: PolylineShape(_parent)
{
  this->odeMesh = new PluginMesh();
}

//////////////////////////////////////////////////
PluginPolylineShape::~PluginPolylineShape()
{
  delete this->odeMesh;
}

//////////////////////////////////////////////////
void PluginPolylineShape::Update()
{
  this->odeMesh->Update();
}

//////////////////////////////////////////////////
void PluginPolylineShape::Load(sdf::ElementPtr _sdf)
{
  PolylineShape::Load(_sdf);
}

//////////////////////////////////////////////////
void PluginPolylineShape::Init()
{
  PolylineShape::Init();
  if (!this->mesh)
  {
    gzerr << "Unable to create polyline in plugin. Mesh pointer is null.\n";
    return;
  }

  this->odeMesh->Init(this->mesh,
      boost::static_pointer_cast<PluginCollision>(this->collisionParent),
      math::Vector3(1, 1, 1));
}
