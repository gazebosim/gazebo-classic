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
#include "common/BVHLoader.hh"

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
  this->skinSkeleton = NULL;
  this->animationSkeleton = NULL;
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

  sdf::ElementPtr skinSdf = _sdf->GetOrCreateElement("skin");
  this->skinFile = skinSdf->GetValueString("filename");
  this->skinScale = skinSdf->GetValueDouble("scale");

  sdf::ElementPtr animSdf = _sdf->GetOrCreateElement("animation");
  this->loop = animSdf->GetValueBool("loop");
  this->timeScale = animSdf->GetValueDouble("time_scale");
  this->startDelay = animSdf->GetValueDouble("delay_start");

  this->useExternalAnim = animSdf->HasElement("external");
  sdf::ElementPtr externAnimSdf = animSdf->GetOrCreateElement("external");
  this->animationFile = externAnimSdf->GetValueString("filename");
  this->animationScale = externAnimSdf->GetValueDouble("scale");

  MeshManager::Instance()->Load(this->skinFile);

  if (MeshManager::Instance()->HasMesh(this->skinFile))
  {
    this->mesh = MeshManager::Instance()->GetMesh(this->skinFile);
    if (!this->mesh->HasSkeleton())
      gzthrow("Collada file does not contain skeletal animation.");
    this->skinSkeleton = mesh->GetSkeleton();
    /// create the link sdfs for the model
    NodeMap nodes = this->skinSkeleton->GetNodes();

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
            this->AddSphereVisual(linkSdf, bone->GetName() + "_visual",
                            math::Pose(), 0.02, "Gazebo/Yellow", Color::Yellow);
        }
        else
          this->AddSphereVisual(linkSdf, bone->GetName() + "_visual",
                            math::Pose(), 0.02, "Gazebo/Red", Color::Red);

      for (unsigned int i = 0; i < bone->GetChildCount(); i++)
      {
        SkeletonNode *curChild = bone->GetChild(i);

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

    if (this->useExternalAnim)
    {
      std::string extension =
            this->animationFile.substr(this->animationFile.rfind(".") + 1,
            this->animationFile.size());

      if (extension == "bvh")
      {
        BVHLoader loader;
        this->animationSkeleton = loader.Load(this->animationFile,
                                                this->animationScale);
      }
      else
        if (extension == "dae")
        {
          MeshManager::Instance()->Load(this->animationFile);
          const Mesh *animMesh = NULL;
          if (MeshManager::Instance()->HasMesh(this->animationFile))
            animMesh = MeshManager::Instance()->GetMesh(this->animationFile);
          if (!animMesh || !animMesh->HasSkeleton())
            gzerr << "Failed to load animation.";
          else
            this->animationSkeleton = animMesh->GetSkeleton();
        }

      if (!this->animationSkeleton ||
            this->animationSkeleton->GetNumAnimations() == 0)
        gzerr << "Failed to load animation.";
      else
      {
        bool compatible = true;
        if (this->skinSkeleton->GetNumNodes() !=
                this->animationSkeleton->GetNumNodes())
          compatible = false;
        else
          for (unsigned int i = 0; i < this->skinSkeleton->GetNumNodes(); i++)
          {
            SkeletonNode *skinNode = this->skinSkeleton->GetNodeByHandle(i);
            SkeletonNode *animNode =
                                this->animationSkeleton->GetNodeByHandle(i);
            if (animNode->GetChildCount() != skinNode->GetChildCount())
              compatible = false;
            else
              this->skelTranslator[animNode->GetName()] = skinNode->GetName();
          }
        if (!compatible)
        {
          gzerr << "Skin and animation skeletons are not compatible.\n";
          this->animationSkeleton = NULL;
        }
        else
          this->skelAnimation =
                   this->animationSkeleton->GetAnimationList().begin()->second;
      }
    }
    else
    {
      this->animationSkeleton = this->skinSkeleton;
      for (unsigned int i = 0; i < this->skinSkeleton->GetNumNodes(); i++)
        this->skelTranslator[this->skinSkeleton->GetNodeByHandle(i)->GetName()]
          = this->skinSkeleton->GetNodeByHandle(i)->GetName();
      this->skelAnimation =
        this->skinSkeleton->GetAnimationList().begin()->second;
    }

    /// we are ready to load the links
    Model::Load(_sdf);
    this->bonePosePub = this->node->Advertise<msgs::PoseAnimation>(
                                       "~/skeleton_pose/info", 10);
  }
}

//////////////////////////////////////////////////
void Actor::Init()
{
  this->prevSkelAnim = this->world->GetSimTime();
}


///////////////////////////////////////////////////
void Actor::Update()
{
  if (!this->animationSkeleton)
    return;

  double timeSinceAnimUpdate = (this->world->GetSimTime()
            - this->prevSkelAnim).Double();

  if (timeSinceAnimUpdate < (1.0 / 30.0))
    return;

  msgs::PoseAnimation msg;

  msg.set_model_name(this->visualName);

  for (unsigned int i = 0; i < this->animationSkeleton->GetNumNodes(); i++)
  {
    SkeletonNode *bone = this->animationSkeleton->GetNodeByHandle(i);
    SkeletonNode *parentBone = bone->GetParent();
    math::Matrix4 transform(math::Matrix4::IDENTITY);
    if (this->skelAnimation.find(bone->GetId()) != this->skelAnimation.end())
    {
      NodeAnimation *anim = &this->skelAnimation[bone->GetId()];
      double start = anim->begin()->first * this->timeScale;
      double end = anim->rbegin()->first * this->timeScale;
      double duration = end - start;
      double frameTime = fmod(this->world->GetSimTime().Double(), duration);

      NodeAnimation::iterator next = anim->begin();
      NodeAnimation::iterator prev = anim->end();
      while ((next->first * this->timeScale) < frameTime)
      {
        prev = next;
        ++next;
      }
      double prevTime = prev->first * this->timeScale;
      math::Matrix4 prevTransform = prev->second;
      if (next != anim->end())
      {
        double nextTime = next->first * this->timeScale;
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

    LinkPtr currentLink = this->GetChildLink(
                            this->skelTranslator[bone->GetName()]);
    if (parentBone)
    {
      if (bone->GetChildCount() > 0)
      {
        math::Pose bonePose;
        bonePose.pos = transform.GetTranslation();
        bonePose.rot = transform.GetRotation();
        msgs::Pose *msg_pose = msg.add_pose();
        msg_pose->set_name(this->skelTranslator[bone->GetName()]);
        msg_pose->mutable_position()->CopyFrom(msgs::Convert(bonePose.pos));
        msg_pose->mutable_orientation()->CopyFrom(msgs::Convert(bonePose.rot));
      }

      LinkPtr parentLink = this->GetChildLink(
                              this->skelTranslator[parentBone->GetName()]);
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
  meshSdf->GetAttribute("filename")->Set(this->skinFile);

  /// use a material with shading for now
  sdf::ElementPtr matSdf = visualSdf->GetOrCreateElement("material");
  matSdf->GetAttribute("script")->Set("Gazebo/Blue");
  sdf::ElementPtr colorSdf = matSdf->GetOrCreateElement("ambient");
  colorSdf->GetAttribute("rgba")->Set(Color::Blue);
}
