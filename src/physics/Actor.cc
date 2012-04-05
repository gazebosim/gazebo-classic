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
#include "common/Skeleton.hh"

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
  std::string filename = _sdf->GetValueString("filename");

  MeshManager::Instance()->Load(filename);

  if (MeshManager::Instance()->HasMesh(filename))
  {
    const Mesh *mesh = MeshManager::Instance()->GetMesh(filename);

    if (!mesh->HasSkeleton())
      gzthrow("Collada file does not contain skeletal animation.");

    NodeMap nodes = mesh->GetSkeleton()->GetNodes();

    for (NodeMapIter iter = nodes.begin(); iter != nodes.end(); ++iter)
    {
      SkeletonNode* bone = iter->second;

      if (!bone->IsJoint())
        continue;

      sdf::ElementPtr linkSdf;
      if (bone->IsRootNode())
        linkSdf = _sdf->GetOrCreateElement("link");
      else
        linkSdf = _sdf->AddElement("link");

      linkSdf->GetAttribute("name")->Set(bone->GetName());
      linkSdf->GetAttribute("gravity")->Set(false);
      sdf::ElementPtr linkPose = linkSdf->GetOrCreateElement("origin");
      math::Pose pose(bone->GetWorldTransform().GetTranslation(),
                      bone->GetWorldTransform().GetRotation());
      linkPose->GetAttribute("pose")->Set(pose);

      /// FIXME hardcoded inertia of a sphere with mass 1.0 and radius 0.05
      sdf::ElementPtr inertialSdf = linkSdf->GetOrCreateElement("inertial");
      inertialSdf->GetAttribute("mass")->Set(1.0);
      sdf::ElementPtr tensorSdf = inertialSdf->GetOrCreateElement("inertia");
      tensorSdf->GetAttribute("ixx")->Set(0.01);
      tensorSdf->GetAttribute("ixy")->Set(0.00);
      tensorSdf->GetAttribute("ixz")->Set(0.00);
      tensorSdf->GetAttribute("iyy")->Set(0.01);
      tensorSdf->GetAttribute("iyz")->Set(0.00);
      tensorSdf->GetAttribute("izz")->Set(0.01);

      /// FIXME hardcoded collision to sphere with radius 0.05
      sdf::ElementPtr collisionSdf = linkSdf->GetOrCreateElement("collision");
      collisionSdf->GetAttribute("name")->Set(bone->GetName()+"_collision");
      sdf::ElementPtr geomColSdf = collisionSdf->GetOrCreateElement("geometry");
      sdf::ElementPtr sphereColSdf = geomColSdf->GetOrCreateElement("sphere");
      sphereColSdf->GetAttribute("radius")->Set(0.05);

      /// FIXME hardcoded visual to red sphere with radius 0.05
      sdf::ElementPtr visualSdf = linkSdf->GetOrCreateElement("visual");
      visualSdf->GetAttribute("name")->Set(bone->GetName()+"_visual");
      sdf::ElementPtr geomVisSdf = visualSdf->GetOrCreateElement("geometry");
      sdf::ElementPtr sphereVisSdf = geomVisSdf->GetOrCreateElement("sphere");
      sphereVisSdf->GetAttribute("radius")->Set(0.05);
      sdf::ElementPtr matSdf = visualSdf->GetOrCreateElement("material");
      matSdf->GetAttribute("script")->Set("Gazebo/Red");
      sdf::ElementPtr colorSdf = matSdf->GetOrCreateElement("ambient");
      colorSdf->GetAttribute("rgba")->Set(Color::Red);
    }
  }
  else
    std::cerr << "no mesh\n";

  Model::Load(_sdf);
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
void Actor::UpdateParameters(sdf::ElementPtr _sdf)
{
//  Model::UpdateParameters(_sdf);
}

//////////////////////////////////////////////////
const sdf::ElementPtr Actor::GetSDF()
{
  return Model::GetSDF();
}

