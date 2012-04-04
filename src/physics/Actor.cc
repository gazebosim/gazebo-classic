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
/* Desc: Base class for all models.
 * Author: Nathan Koenig and Andrew Howard
 * Date: 8 May 2003
 */
#include <boost/thread/recursive_mutex.hpp>
#include <sstream>

#include "common/KeyFrame.hh"
#include "common/Animation.hh"
#include "common/Plugin.hh"
#include "common/Events.hh"
#include "common/Exception.hh"
#include "common/Console.hh"
#include "common/CommonTypes.hh"
#include "common/MeshManager.hh"
#include "common/Mesh.hh"

#include "physics/Joint.hh"
#include "physics/Link.hh"
#include "physics/World.hh"
#include "physics/PhysicsEngine.hh"
#include "physics/Actor.hh"

#include "transport/Node.hh"

using namespace gazebo;
using namespace physics;
using namespace common;

//////////////////////////////////////////////////
Actor::Actor(BasePtr _parent)
  : Model(_parent)
{
  this->AddType(ACTOR);
}

//////////////////////////////////////////////////
Actor::~Actor()
{
}

//////////////////////////////////////////////////
void Actor::Load(sdf::ElementPtr _sdf)
{
  gzwarn << "Actor::Load()\n";

  std::string filename = _sdf->GetValueString("filename");

  MeshManager::Instance()->Load(filename);

  if (MeshManager::Instance()->HasMesh(filename))
  {
    /*const Mesh *mesh =*/ MeshManager::Instance()->GetMesh(filename);
  }
  else
    std::cerr << "no mesh\n";
}

//////////////////////////////////////////////////
void Actor::Init()
{
}


//////////////////////////////////////////////////
void Actor::Update()
{
//  this->updateMutex->lock();

//  this->updateMutex->unlock();
}

//////////////////////////////////////////////////
void Actor::Fini()
{
  Model::Fini();
}

//////////////////////////////////////////////////
void Actor::UpdateParameters(sdf::ElementPtr /*_sdf*/)
{
//  Model::UpdateParameters(_sdf);
}

//////////////////////////////////////////////////
const sdf::ElementPtr Actor::GetSDF()
{
  return Model::GetSDF();
}

