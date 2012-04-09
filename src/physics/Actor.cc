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
#include "physics/Model.hh"
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

    /// create the link sdfs for the model
    NodeMap nodes = mesh->GetSkeleton()->GetNodes();

    for (NodeMapIter iter = nodes.begin(); iter != nodes.end(); ++iter)
    {
      SkeletonNode* bone = iter->second;

      //  if (!bone->IsJoint())
      //    continue;

      sdf::ElementPtr linkSdf;
      if (bone->IsRootNode())
        linkSdf = _sdf->GetOrCreateElement("link");
      else
        linkSdf = _sdf->AddElement("link");

      linkSdf->GetAttribute("name")->Set(bone->GetName());
      linkSdf->GetAttribute("gravity")->Set(false);
      sdf::ElementPtr linkPose = linkSdf->GetOrCreateElement("origin");
      math::Pose pose(bone->GetModelTransform().GetTranslation(),
                      bone->GetModelTransform().GetRotation());
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

    /// we are ready to load the links
    Model::Load(_sdf);

    /// create the pose animations based on the skeletal animations
    std::map<std::string, PoseAnimationPtr> animations;
    SkeletonAnimation skelAnim =
        mesh->GetSkeleton()->GetAnimationList().begin()->second;

    for (unsigned int i = 0; i < mesh->GetSkeleton()->GetNumNodes(); i++)
    {
      SkeletonNode *skelNode = mesh->GetSkeleton()->GetNodeByHandle(i);
      if (skelAnim.find(skelNode->GetId()) != skelAnim.end())
      {
        NodeAnimation *nodeAnim = &skelAnim[skelNode->GetId()];
        double startTime = (*nodeAnim).begin()->first;
        double endTime = (*nodeAnim).rbegin()->first;
        double duration = endTime - startTime;

        PoseAnimationPtr anim(
            new PoseAnimation(skelNode->GetName() + "_anim", duration, true));

        for (NodeAnimation::iterator iter = (*nodeAnim).begin();
                iter != (*nodeAnim).end(); ++iter)
        {
          double frameTime = iter->first;
          math::Matrix4 trans = iter->second;

          PoseKeyFrame *key;

          if (skelNode->IsRootNode())
          {
            key = anim->CreateKeyFrame(frameTime);
            key->SetTranslation(trans.GetTranslation());
            key->SetRotation(trans.GetRotation());
          }
          else
          {
            SkeletonNode *nodeParent = skelNode->GetParent();

            if (animations.find(nodeParent->GetName()) != animations.end())
            {
              PoseAnimationPtr parentAnim = animations[nodeParent->GetName()];
              parentAnim->SetTime(frameTime);
              PoseKeyFrame parentFrame(frameTime);
              parentAnim->GetInterpolatedKeyFrame(parentFrame);
              math::Matrix4 parentTrans =
                  parentFrame.GetRotation().GetAsMatrix4();

              parentTrans.SetTranslate(parentFrame.GetTranslation());

              math::Matrix4 frameTrans = parentTrans * trans;
              key = anim->CreateKeyFrame(frameTime);
              key->SetTranslation(frameTrans.GetTranslation());
              key->SetRotation(frameTrans.GetRotation());
            }
            else
            {
              std::cerr << "parent not found in animation list.\n";
            }
          }
        }
        animations[skelNode->GetName()] = anim;
        LinkPtr link = this->GetChildLink(skelNode->GetName());
        link->SetAnimation(anim);
      }
      else /* if node is not skeleton animated */
      {
        /// we need the transform from this node to the first animated node in
        /// its parent node structure
        SkeletonNode *nodeParent = skelNode->GetParent();
        math::Matrix4 trans = skelNode->GetTransform();
        while (animations.find(nodeParent->GetName()) == animations.end())
        {
          trans = nodeParent->GetTransform() * trans;
          nodeParent = nodeParent->GetParent();
        }
        PoseAnimationPtr parentAnim = animations[nodeParent->GetName()];
        PoseAnimationPtr anim( new PoseAnimation(skelNode->GetName() + "_anim",
            parentAnim->GetLength(), true));

        for (unsigned int j = 0; j < parentAnim->GetNumKeyFrames(); j++)
        {
          PoseKeyFrame *parentFrame =
              reinterpret_cast<PoseKeyFrame*>(parentAnim->GetKeyFrame(j));

          math::Matrix4 parentTrans =
            parentFrame->GetRotation().GetAsMatrix4();
          parentTrans.SetTranslate(parentFrame->GetTranslation());
          double frameTime = parentFrame->GetTime();
          math::Matrix4 frameTrans = parentTrans * trans;

          PoseKeyFrame *key;
          key = anim->CreateKeyFrame(frameTime);
          key->SetTranslation(frameTrans.GetTranslation());
          key->SetRotation(frameTrans.GetRotation());
        }
        animations[skelNode->GetName()] = anim;
        LinkPtr link = this->GetChildLink(skelNode->GetName());
        link->SetAnimation(anim);
      }
    }
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

