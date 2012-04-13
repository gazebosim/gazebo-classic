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
#include <msgs/msgs.h>

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
  this->bonePosePub.reset();
}

//////////////////////////////////////////////////
void Actor::Load(sdf::ElementPtr _sdf)
{
  std::string name = _sdf->GetValueString("name");
  this->fileName = _sdf->GetValueString("filename");
  this->timeFactor = _sdf->GetValueDouble("time_factor");

  MeshManager::Instance()->Load(this->fileName);

  if (MeshManager::Instance()->HasMesh(this->fileName))
  {
    this->mesh = MeshManager::Instance()->GetMesh(this->fileName);
    if (!this->mesh->HasSkeleton())
      gzthrow("Collada file does not contain skeletal animation.");
    this->skeleton = mesh->GetSkeleton();
    this->skelAnimation = this->skeleton->GetAnimationList().begin()->second;

    /// create the link sdfs for the model
    NodeMap nodes = this->skeleton->GetNodes();

    sdf::ElementPtr linkSdf;
    linkSdf = _sdf->GetOrCreateElement("link");
    linkSdf->GetAttribute("name")->Set(this->GetName() + "_origin");
    linkSdf->GetAttribute("gravity")->Set(false);
    sdf::ElementPtr linkPose = linkSdf->GetOrCreateElement("origin");

    this->AddSphereInertia(linkSdf, math::Pose(), 1.0, 0.01);
    this->AddSphereCollision(linkSdf, this->GetName() + "_origin_col",
                                             math::Pose(), 0.02);
    this->AddSphereVisual(linkSdf, this->GetName() + "_origin_vis",
                    math::Pose(), 0.05, "Gazebo/White", Color::White);


    for (NodeMapIter iter = nodes.begin(); iter != nodes.end(); ++iter)
    {
      SkeletonNode* bone = iter->second;

      linkSdf = _sdf->AddElement("link");

      linkSdf->GetAttribute("name")->Set(bone->GetName());
      linkSdf->GetAttribute("gravity")->Set(false);
      linkPose = linkSdf->GetOrCreateElement("origin");
      math::Pose pose(bone->GetModelTransform().GetTranslation(),
                      bone->GetModelTransform().GetRotation());
      linkPose->GetAttribute("pose")->Set(pose);

      /// FIXME hardcoded inertia of a sphere with mass 1.0 and radius 0.01
      this->AddSphereInertia(linkSdf, math::Pose(), 1.0, 0.01);

      /// FIXME hardcoded collision to sphere with radius 0.02
      this->AddSphereCollision(linkSdf, bone->GetName() + "_collision",
                       math::Pose(), 0.02);

      /// FIXME hardcoded visual to red sphere with radius 0.02
      if (bone->IsRootNode())
      {
        this->AddSphereVisual(linkSdf, bone->GetName() + "_visual",
                            math::Pose(), 0.02, "Gazebo/Blue", Color::Blue);
        this->AddActorVisual(linkSdf, name + "_visual", math::Pose());
        this->visualName = name + "::" + bone->GetName()
                             + "::" + name + "_visual";
      }
      else
        if (bone->GetChildCount() == 0)
        {
          /// FIXME hack to hide the gaze link visual for now
          if (bone->GetName().find("SEH") == std::string::npos)
            this->AddSphereVisual(linkSdf, bone->GetName() + "_visual",
                            math::Pose(), 0.02, "Gazebo/Yellow", Color::Yellow);
        }
        else
          this->AddSphereVisual(linkSdf, bone->GetName() + "_visual",
                            math::Pose(), 0.02, "Gazebo/Red", Color::Red);

      for (unsigned int i = 0; i < bone->GetChildCount(); i++)
      {
        SkeletonNode *curChild = bone->GetChild(i);

        /// FIXME hack to hide the gaze link visual for now
        if (curChild->GetName().find("SEH") != std::string::npos)
          continue;

        math::Vector3 r = curChild->GetTransform().GetTranslation();
        math::Vector3 linkPos = math::Vector3(r.x / 2.0, r.y / 2.0, r.z / 2.0);
        double length = r.GetLength();
        double theta = atan2(r.y, r.x);

        double phi = acos(r.z / length);
        math::Pose bonePose(linkPos, math::Quaternion(0.0, phi, theta));

        this->AddBoxVisual(linkSdf, bone->GetName() + "_" + curChild->GetName()
          + "_link", bonePose, math::Vector3(0.02, 0.02, length),
          "Gazebo/Green", Color::Green);
      }
    }

    /// we are ready to load the links
    Model::Load(_sdf);
    this->bonePosePub = this->node->Advertise<msgs::PoseAnimation>(
                                       "~/skeleton_pose/info", 10);
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

  msgs::PoseAnimation msg;

  msg.set_model_name(this->visualName);

  for (unsigned int i = 0; i < this->skeleton->GetNumNodes(); i++)
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
        math::Quaternion curQ;

        math::Vector3 curPos =
              math::Vector3(prevPos.x + (nextPos.x - prevPos.x) * t,
                           prevPos.y + (nextPos.y - prevPos.y) * t,
                           prevPos.z + (nextPos.z - prevPos.z) * t);
        curQ = math::Quaternion::Slerp(t, prevQ, nextQ, true);

        transform = curQ.GetAsMatrix4();
        transform.SetTranslate(curPos);
      }
      else
        transform = prevTransform;
    }
    else
      transform = bone->GetTransform();

    LinkPtr currentLink = this->GetChildLink(bone->GetName());
    if (parentBone)
    {
      if (bone->GetChildCount() > 0)
      {
        math::Pose bonePose;
        bonePose.pos = transform.GetTranslation();
        bonePose.rot = transform.GetRotation();
        msgs::Pose *msg_pose = msg.add_pose();
        msg_pose->set_name(bone->GetName());
        msg_pose->mutable_position()->CopyFrom(msgs::Convert(bonePose.pos));
        msg_pose->mutable_orientation()->CopyFrom(msgs::Convert(bonePose.rot));
      }

      LinkPtr parentLink = this->GetChildLink(parentBone->GetName());
      math::Pose parentPose = parentLink->GetWorldPose();
      math::Matrix4 parentTrans(math::Matrix4::IDENTITY);
      parentTrans = parentPose.rot.GetAsMatrix4();
      parentTrans.SetTranslate(parentPose.pos);
      transform = parentTrans * transform;
    }
    currentLink->SetWorldPose(transform.GetAsPose());
  }

  if (this->bonePosePub && this->bonePosePub->HasConnections())
    this->bonePosePub->Publish(msg);

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

//////////////////////////////////////////////////
void Actor::AddSphereInertia(sdf::ElementPtr linkSdf, math::Pose pose,
            double mass, double radius)
{
  double ixx = 2.0 * mass * radius * radius / 5.0;
  sdf::ElementPtr inertialSdf = linkSdf->GetOrCreateElement("inertial");
  sdf::ElementPtr inertialPoseSdf = inertialSdf->GetOrCreateElement("origin");
  inertialPoseSdf->GetAttribute("pose")->Set(pose);
  inertialSdf->GetAttribute("mass")->Set(mass);
  sdf::ElementPtr tensorSdf = inertialSdf->GetOrCreateElement("inertia");
  tensorSdf->GetAttribute("ixx")->Set(ixx);
  tensorSdf->GetAttribute("ixy")->Set(0.00);
  tensorSdf->GetAttribute("ixz")->Set(0.00);
  tensorSdf->GetAttribute("iyy")->Set(ixx);
  tensorSdf->GetAttribute("iyz")->Set(0.00);
  tensorSdf->GetAttribute("izz")->Set(ixx);
}

//////////////////////////////////////////////////
void Actor::AddSphereCollision(sdf::ElementPtr linkSdf, std::string name,
            math::Pose pose, double radius)
{
  sdf::ElementPtr collisionSdf = linkSdf->GetOrCreateElement("collision");
  collisionSdf->GetAttribute("name")->Set(name);
  sdf::ElementPtr collPoseSdf = collisionSdf->GetOrCreateElement("origin");
  collPoseSdf->GetAttribute("pose")->Set(pose);
  sdf::ElementPtr geomColSdf = collisionSdf->GetOrCreateElement("geometry");
  sdf::ElementPtr sphereColSdf = geomColSdf->GetOrCreateElement("sphere");
  sphereColSdf->GetAttribute("radius")->Set(radius);
}

//////////////////////////////////////////////////
void Actor::AddSphereVisual(sdf::ElementPtr linkSdf, std::string name,
            math::Pose pose, double radius, std::string material, Color ambient)
{
  sdf::ElementPtr visualSdf = linkSdf->GetOrCreateElement("visual");
  visualSdf->GetAttribute("name")->Set(name);
  sdf::ElementPtr visualPoseSdf = visualSdf->GetOrCreateElement("origin");
  visualPoseSdf->GetAttribute("pose")->Set(pose);
  sdf::ElementPtr geomVisSdf = visualSdf->GetOrCreateElement("geometry");
  sdf::ElementPtr sphereVisSdf = geomVisSdf->GetOrCreateElement("sphere");
  sphereVisSdf->GetAttribute("radius")->Set(radius);
  sdf::ElementPtr matSdf = visualSdf->GetOrCreateElement("material");
  matSdf->GetAttribute("script")->Set(material);
  sdf::ElementPtr colorSdf = matSdf->GetOrCreateElement("ambient");
  colorSdf->GetAttribute("rgba")->Set(ambient);
}

//////////////////////////////////////////////////
void Actor::AddBoxVisual(sdf::ElementPtr linkSdf, std::string name,
    math::Pose pose, math::Vector3 size, std::string material, Color ambient)
{
  sdf::ElementPtr visualSdf = linkSdf->AddElement("visual");
  visualSdf->GetAttribute("name")->Set(name);
  sdf::ElementPtr visualPoseSdf = visualSdf->GetOrCreateElement("origin");
  visualPoseSdf->GetAttribute("pose")->Set(pose);
  sdf::ElementPtr geomVisSdf = visualSdf->GetOrCreateElement("geometry");
  sdf::ElementPtr boxSdf = geomVisSdf->GetOrCreateElement("box");
  boxSdf->GetAttribute("size")->Set(size);
  sdf::ElementPtr matSdf = visualSdf->GetOrCreateElement("material");
  matSdf->GetAttribute("script")->Set(material);
  sdf::ElementPtr colorSdf = matSdf->GetOrCreateElement("ambient");
  colorSdf->GetAttribute("rgba")->Set(ambient);
}

//////////////////////////////////////////////////
void Actor::AddActorVisual(sdf::ElementPtr linkSdf, std::string name,
    math::Pose pose)
{
  sdf::ElementPtr visualSdf = linkSdf->AddElement("visual");
  visualSdf->GetAttribute("name")->Set(name);
  sdf::ElementPtr visualPoseSdf = visualSdf->GetOrCreateElement("origin");
  visualPoseSdf->GetAttribute("pose")->Set(pose);
  sdf::ElementPtr geomVisSdf = visualSdf->GetOrCreateElement("geometry");
  sdf::ElementPtr meshSdf = geomVisSdf->GetOrCreateElement("mesh");
  meshSdf->GetAttribute("filename")->Set(this->fileName);

  /// use a material with shading for now
  sdf::ElementPtr matSdf = visualSdf->GetOrCreateElement("material");
  matSdf->GetAttribute("script")->Set("Gazebo/Blue");
  sdf::ElementPtr colorSdf = matSdf->GetOrCreateElement("ambient");
  colorSdf->GetAttribute("rgba")->Set(Color::Blue);
}
