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

#include "physics/World.hh"
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
  this->mesh = NULL;
  this->skeleton = NULL;
}

//////////////////////////////////////////////////
Actor::~Actor()
{
  this->skelAnimation.clear();
}

//////////////////////////////////////////////////
void Actor::Load(sdf::ElementPtr _sdf)
{
  this->fileName = _sdf->GetValueString("filename");
  this->timeFactor = _sdf->GetValueDouble("time_factor");

  MeshManager::Instance()->Load(this->fileName);

  if (MeshManager::Instance()->HasMesh(this->fileName))
  {
    this->mesh = MeshManager::Instance()->GetMesh(this->fileName);

    if (!this->mesh->HasSkeleton())
      gzthrow("Collada file does not contain skeletal animation.");

    this->skeleton = mesh->GetSkeleton();

    /// create the link sdfs for the model
    NodeMap nodes = this->skeleton->GetNodes();

    for (NodeMapIter iter = nodes.begin(); iter != nodes.end(); ++iter)
    {
      SkeletonNode* bone = iter->second;

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

      /// FIXME hardcoded inertia of a sphere with mass 1.0 and radius 0.01
      sdf::ElementPtr inertialSdf = linkSdf->GetOrCreateElement("inertial");
      inertialSdf->GetAttribute("mass")->Set(1.0);
      sdf::ElementPtr tensorSdf = inertialSdf->GetOrCreateElement("inertia");
      tensorSdf->GetAttribute("ixx")->Set(0.00004);
      tensorSdf->GetAttribute("ixy")->Set(0.00);
      tensorSdf->GetAttribute("ixz")->Set(0.00);
      tensorSdf->GetAttribute("iyy")->Set(0.00004);
      tensorSdf->GetAttribute("iyz")->Set(0.00);
      tensorSdf->GetAttribute("izz")->Set(0.00004);

      /// FIXME hardcoded collision to sphere with radius 0.02
      sdf::ElementPtr collisionSdf = linkSdf->GetOrCreateElement("collision");
      collisionSdf->GetAttribute("name")->Set(bone->GetName()+"_collision");
      sdf::ElementPtr geomColSdf = collisionSdf->GetOrCreateElement("geometry");
      sdf::ElementPtr sphereColSdf = geomColSdf->GetOrCreateElement("sphere");
      sphereColSdf->GetAttribute("radius")->Set(0.02);

      /// FIXME hardcoded visual to red sphere with radius 0.02
      sdf::ElementPtr visualSdf = linkSdf->GetOrCreateElement("visual");
      visualSdf->GetAttribute("name")->Set(bone->GetName()+"_visual");
      sdf::ElementPtr geomVisSdf = visualSdf->GetOrCreateElement("geometry");
      sdf::ElementPtr sphereVisSdf = geomVisSdf->GetOrCreateElement("sphere");
      sphereVisSdf->GetAttribute("radius")->Set(0.02);
      sdf::ElementPtr matSdf = visualSdf->GetOrCreateElement("material");
      sdf::ElementPtr colorSdf = matSdf->GetOrCreateElement("ambient");
      if (bone->IsRootNode())
      {
        matSdf->GetAttribute("script")->Set("Gazebo/Blue");
        colorSdf->GetAttribute("rgba")->Set(Color::Blue);
      }
      else
        if (bone->GetChildCount() == 0)
        {
          matSdf->GetAttribute("script")->Set("Gazebo/Yellow");
          colorSdf->GetAttribute("rgba")->Set(Color::Yellow);
        }
        else
        {
          matSdf->GetAttribute("script")->Set("Gazebo/Red");
          colorSdf->GetAttribute("rgba")->Set(Color::Red);
        }

      for (unsigned int i = 0; i < bone->GetChildCount(); i++)
      {
        SkeletonNode *curChild = bone->GetChild(i);

        math::Vector3 r = curChild->GetTransform().GetTranslation();
        math::Vector3 linkPos = math::Vector3(r.x / 2.0, r.y / 2.0, r.z / 2.0);
        double length = r.GetLength();
        double theta = atan2(r.y, r.x);

        double phi = acos( r.z / length);
        math::Pose bonePose(linkPos, math::Quaternion(0.0, phi, theta));

        visualSdf = linkSdf->AddElement("visual");
        visualSdf->GetAttribute("name")->Set(curChild->GetName()+"_link_vis");
        sdf::ElementPtr visualPoseSdf = visualSdf->GetOrCreateElement("origin");
        visualPoseSdf->GetAttribute("pose")->Set(bonePose);
        geomVisSdf = visualSdf->GetOrCreateElement("geometry");
        sdf::ElementPtr boxSdf = geomVisSdf->GetOrCreateElement("box");
        boxSdf->GetAttribute("size")->Set(math::Vector3(0.02, 0.02, length));
        matSdf = visualSdf->GetOrCreateElement("material");
        matSdf->GetAttribute("script")->Set("Gazebo/Green");
        colorSdf = matSdf->GetOrCreateElement("ambient");
        colorSdf->GetAttribute("rgba")->Set(Color::Green);
      }

//       if (!bone->IsRootNode())
//       {
//         unsigned int jointCount = 3;
//         if (!bone->IsJoint())
//           jointCount = 1;
//         for (unsigned int i = 0; i < jointCount; i++)
//         {
//           math::Vector3 axis = math::Vector3(0.0, 0.0, 1.0);
//           std::string name = "_joint";
//           if (jointCount == 3)
//           {
//             switch (i)
//             {
//               case 0: axis = math::Vector3(0.0, 0.0, 1.0);
//                       name = "_joint_Z";
//                       break;
//               case 1: axis = math::Vector3(1.0, 0.0, 0.0);
//                       name = "_joint_X";
//                       break;
//               default: axis = math::Vector3(0.0, 1.0, 0.0);
//                        name = "_joint_Y";
//                        break;
//             }
//           }
//           sdf::ElementPtr jointSdf = _sdf->AddElement("joint");
//           jointSdf->GetAttribute("name")->Set(bone->GetName() + name);
//           jointSdf->GetAttribute("type")->Set("revolute");
//           sdf::ElementPtr parentSdf = jointSdf->GetOrCreateElement("parent");
//           parentSdf->GetAttribute("link")->Set(bone->GetParent()->GetName());
//           sdf::ElementPtr childSdf = jointSdf->GetOrCreateElement("child");
//           childSdf->GetAttribute("link")->Set(bone->GetName());
//           sdf::ElementPtr axisSdf = jointSdf->GetOrCreateElement("axis");
//           axisSdf->GetAttribute("xyz")->Set(axis);
//         }
//      }
    }

    /// we are ready to load the links
    Model::Load(_sdf);

    SkeletonAnimation skelAnim =
            this->skeleton->GetAnimationList().begin()->second;

    PoseAnimationPtr baseAnimation;
    SkeletonNode *skelNode = this->skeleton->GetNodeByHandle(0);
    if (skelAnim.find(skelNode->GetId()) != skelAnim.end())
    {
      NodeAnimation *nodeAnim = &skelAnim[skelNode->GetId()];
      double startTime = (*nodeAnim).begin()->first;
      double endTime = (*nodeAnim).rbegin()->first;
      double duration = (endTime - startTime) * this->timeFactor;

      baseAnimation.reset(new PoseAnimation(skelNode->GetName() + "_anim",
                duration, true));

      for (NodeAnimation::iterator iter = (*nodeAnim).begin();
              iter != (*nodeAnim).end(); ++iter)
      {
        double frameTime = iter->first * this->timeFactor;
        math::Matrix4 trans = iter->second;

        PoseKeyFrame *key;
        key = baseAnimation->CreateKeyFrame(frameTime);
        key->SetTranslation(trans.GetTranslation());
        key->SetRotation(trans.GetRotation());
      }
    }


//     std::map<std::string, NumericAnimationPtr> jointAnims;
//     for (unsigned int i = 1; i < this->skeleton->GetNumNodes(); i++)
//     {
//       *skelNode = this->skeleton->GetNodeByHandle(i);
//       if (skelAnim.find(skelNode->GetId()) != skelAnim.end())
//       {
//         NodeAnimation *nodeAnim = &skelAnim[skelNode->GetId()];
//         double startTime = (*nodeAnim).begin()->first;
//         double endTime = (*nodeAnim).rbegin()->first;
//         double duration = (endTime - startTime) * this->timeFactor;

//         NumericAnimationPtr jAnimZ, jAnimY, jAnimX;
//         double jAnimZVal = 0.0;
//         double jAnimYVal = 0.0;
//         double jAnimXVal = 0.0;
//         bool usejAnimZ = false;
//         bool usejAnimY = false;
//         bool usejAnimX = false;

//         jAnimZ.reset(new NumericAnimation(skelNode->GetName()+"_Zanim",
//                      duration, true));
//         jAnimX.reset(new NumericAnimation(skelNode->GetName()+"_Xanim",
//                      duration, true));
//         jAnimY.reset(new NumericAnimation(skelNode->GetName()+"_Yanim",
//                      duration, true));
//         for (NodeAnimation::iterator iter = (*nodeAnim).begin();
//                 iter != (*nodeAnim).end(); ++iter)
//         {
//           double frameTime = iter->first * this->timeFactor;
//           math::Matrix4 trans = iter->second;

//           math::Vector3 angles = trans.GetRotation().GetAsEuler();
//           NumericKeyFrame *key;
//           key = jAnimZ->CreateKeyFrame(frameTime);
//           key->SetValue(angles.z);
//           key = jAnimX->CreateKeyFrame(frameTime);
//           key->SetValue(angles.x);
//           key = jAnimY->CreateKeyFrame(frameTime);
//           key->SetValue(angles.y);
//           if (iter == (*nodeAnim).begin())
//           {
//             jAnimZVal = angles.z;
//             jAnimYVal = angles.y;
//             jAnimXVal = angles.x;
//           }
//           else
//           {
//             if (!math::equal(angles.z, jAnimZVal))
//               usejAnimZ = true;
//             if (!math::equal(angles.x, jAnimXVal))
//               usejAnimX = true;
//             if (!math::equal(angles.y, jAnimYVal))
//               usejAnimY = true;
//           }

//         }
//         if (usejAnimZ)
//           jointAnims[skelNode->GetName()+"_joint_Z"] = jAnimZ;
//         if (usejAnimX)
//           jointAnims[skelNode->GetName()+"_joint_X"] = jAnimX;
//         if (usejAnimY)
//           jointAnims[skelNode->GetName()+"_joint_Y"] = jAnimY;

//       }
//    }
    //this->SetAnimation(baseAnimation);
    this->skelAnimation = this->skeleton->GetAnimationList().begin()->second;
    //this->SetJointAnimation(jointAnims);
  }
  else
    std::cerr << "no mesh\n";
}

//////////////////////////////////////////////////
void Actor::Init()
{
  this->prevSkelAnim = this->world->GetSimTime();
}


//////////////////////////////////////////////////
void Actor::Update()
{
  if (!this->skeleton)
    return;

  double timeSinceAnimUpdate = (this->world->GetSimTime()
            - this->prevSkelAnim).Double();

  if (timeSinceAnimUpdate < (1.0 / 30.0))
    return;

  /// start at second node, root node animation is applied to whole model
  for (unsigned int i = 1; i < this->skeleton->GetNumNodes(); i++)
  {
    SkeletonNode *bone = this->skeleton->GetNodeByHandle(i);
    SkeletonNode *parentBone = bone->GetParent();
    math::Matrix4 transform(math::Matrix4::IDENTITY);
    if (this->skelAnimation.find(bone->GetId()) != this->skelAnimation.end())
    {
      NodeAnimation *anim = &this->skelAnimation[bone->GetId()];
      double start = anim->begin()->first * this->timeFactor;
      double end = anim->rbegin()->first * this->timeFactor;
      double duration = end - start;
      double frameTime = fmod(this->world->GetSimTime().Double(), duration);

      NodeAnimation::iterator next = anim->begin();
      NodeAnimation::iterator prev = anim->end();
      while ((next->first * this->timeFactor) < frameTime)
      {
        prev = next;
        ++next;
      }
      double prevTime = prev->first * this->timeFactor;
      math::Matrix4 prevTransform = prev->second;
      if (next != anim->end())
      {
        double nextTime = next->first * this->timeFactor;
        math::Matrix4 nextTransform = next->second;
        double t = (frameTime - prevTime) / (nextTime - prevTime);
        math::Vector3 prevPos = prevTransform.GetTranslation();
        math::Vector3 nextPos = nextTransform.GetTranslation();
        math::Quaternion prevQ = prevTransform.GetRotation();
        math::Quaternion nextQ = nextTransform.GetRotation();

        math::Vector3 curPos =
              math::Vector3(prevPos.x + (nextPos.x - prevPos.x) * t,
                           prevPos.y + (nextPos.y - prevPos.y) * t,
                           prevPos.z + (nextPos.z - prevPos.z) * t);
        math::Quaternion curQ = math::Quaternion::Slerp(t, prevQ, nextQ, true);

        transform = curQ.GetAsMatrix4();
        transform.SetTranslate(curPos);
      }
      else
        transform = prevTransform;
    }
    else
      transform = bone->GetTransform();

    LinkPtr parentLink = this->GetChildLink(parentBone->GetName());
    LinkPtr currentLink = this->GetChildLink(bone->GetName());

    math::Matrix4 parentTrans(math::Matrix4::IDENTITY);
    math::Pose parentPose = parentLink->GetRelativePose();
    parentTrans = parentPose.rot.GetAsMatrix4();
    parentTrans.SetTranslate(parentPose.pos);

    transform = parentTrans * transform;
    currentLink->SetRelativePose(transform.GetAsPose());
  }
  this->prevSkelAnim = this->world->GetSimTime();
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

