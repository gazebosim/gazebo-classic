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
#include <msgs/msgs.h>

#include <boost/thread/recursive_mutex.hpp>
#include <sstream>
#include <limits>
#include <algorithm>

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
#include "common/SkeletonAnimation.hh"
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

  sdf::ElementPtr skinSdf = _sdf->GetOrCreateElement("skin");
  this->skinFile = skinSdf->GetValueString("filename");
  this->skinScale = skinSdf->GetValueDouble("scale");

  MeshManager::Instance()->Load(this->skinFile);
  std::string actorName = _sdf->GetValueString("name");

  if (MeshManager::Instance()->HasMesh(this->skinFile))
  {
    this->mesh = MeshManager::Instance()->GetMesh(this->skinFile);
    if (!this->mesh->HasSkeleton())
      gzthrow("Collada file does not contain skeletal animation.");
    this->skeleton = mesh->GetSkeleton();
    this->skeleton->Scale(this->skinScale);
    /// create the link sdfs for the model
    NodeMap nodes = this->skeleton->GetNodes();

    sdf::ElementPtr linkSdf;
    linkSdf = _sdf->GetOrCreateElement("link");
    linkSdf->GetAttribute("name")->Set(actorName + "_origin");
    linkSdf->GetAttribute("gravity")->Set(false);
    sdf::ElementPtr linkPose = linkSdf->GetOrCreateElement("origin");

    this->AddSphereInertia(linkSdf, math::Pose(), 1.0, 0.01);
    this->AddSphereCollision(linkSdf, actorName + "_origin_col",
                                             math::Pose(), 0.02);
    this->AddSphereVisual(linkSdf, actorName + "_origin_vis",
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
        this->AddActorVisual(linkSdf, name + "_visual", pose.GetInverse());
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
        math::Vector3 dir = curChild->GetModelTransform().GetTranslation() -
            bone->GetModelTransform().GetTranslation();
        double length = dir.GetLength();
        double theta = atan2(dir.y, dir.x);
        double phi = acos(dir.z / length);

        math::Pose bonePose(linkPos, math::Quaternion(0.0, phi, theta));
        bonePose.rot = pose.rot.GetInverse() * bonePose.rot;

        this->AddBoxVisual(linkSdf, bone->GetName() + "_" + curChild->GetName()
          + "_link", bonePose, math::Vector3(0.02, 0.02, length),
          "Gazebo/Green", Color::Green);
      }
    }

    sdf::ElementPtr animSdf = _sdf->GetOrCreateElement("animation");

    while (animSdf)
    {
      this->LoadAnimation(animSdf);
      animSdf = animSdf->GetNextElement("animation");
    }

    this->LoadScript(_sdf->GetOrCreateElement("script"));

    /// we are ready to load the links
    Model::Load(_sdf);
    this->bonePosePub = this->node->Advertise<msgs::PoseAnimation>(
                                       "~/skeleton_pose/info", 10);
  }
}

//////////////////////////////////////////////////
void Actor::LoadScript(sdf::ElementPtr _sdf)
{
  this->loop = _sdf->GetValueBool("loop");
  this->startDelay = _sdf->GetValueDouble("delay_start");
  this->autoStart = _sdf->GetValueBool("auto_start");
  this->active = this->autoStart;

  sdf::ElementPtr trajSdf = _sdf->GetOrCreateElement("trajectory");
  while (trajSdf)
  {
    TrajectoryInfo tinfo;
    tinfo.id = trajSdf->GetValueInt("id");
    tinfo.type = trajSdf->GetValueString("type");
    std::vector<TrajectoryInfo>::iterator iter = this->trajInfo.begin();
    while (iter != this->trajInfo.end())
    {
      if (iter->id > tinfo.id)
        break;
      ++iter;
    }

    unsigned int idx = iter - this->trajInfo.begin();
    this->trajInfo.insert(iter, tinfo);

    std::map<double, math::Pose> points;

    sdf::ElementPtr wayptSdf = trajSdf->GetOrCreateElement("waypoint");
    while (wayptSdf)
    {
      points[wayptSdf->GetValueDouble("time")] = wayptSdf->GetValuePose("pose");
      wayptSdf = wayptSdf->GetNextElement("waypoint");
    }

    std::map<double, math::Pose>::reverse_iterator last = points.rbegin();
    std::stringstream animName;
    animName << tinfo.type << "_" << tinfo.id;
    common::PoseAnimation *anim = new common::PoseAnimation(animName.str(),
                                                          last->first, false);
    this->trajInfo[idx].duration = last->first;

    for (std::map<double, math::Pose>::iterator pIter = points.begin();
          pIter != points.end(); ++pIter)
    {
      common::PoseKeyFrame *key;
      if (pIter == points.begin() && !math::equal(pIter->first, 0.0))
      {
        key = anim->CreateKeyFrame(0.0);
        key->SetTranslation(pIter->second.pos);
        key->SetRotation(pIter->second.rot);
      }
      key = anim->CreateKeyFrame(pIter->first);
      key->SetTranslation(pIter->second.pos);
      key->SetRotation(pIter->second.rot);
    }

    this->trajectories.insert(this->trajectories.begin() + idx, anim);

    trajSdf = trajSdf->GetNextElement("trajectory");
  }
  double scriptTime = 0.0;
  for (unsigned int i = 0; i < this->trajInfo.size(); i++)
  {
    this->trajInfo[i].startTime = scriptTime;
    scriptTime += this->trajInfo[i].duration;
    this->trajInfo[i].endTime = scriptTime;
  }
  this->scriptLength = scriptTime;
}

//////////////////////////////////////////////////
void Actor::LoadAnimation(sdf::ElementPtr _sdf)
{
  std::string animName = _sdf->GetValueString("name");

  if (animName == "__default__")
  {
    this->skelAnimation[this->skinFile] =
        this->skeleton->GetAnimation(0);
    std::map<std::string, std::string> skelMap;
    for (unsigned int i = 0; i < this->skeleton->GetNumNodes(); i++)
      skelMap[this->skeleton->GetNodeByHandle(i)->GetName()] =
        this->skeleton->GetNodeByHandle(i)->GetName();
    this->skelNodesMap[this->skinFile] = skelMap;
    this->interpolateX[this->skinFile] = false;
  }
  else
  {
    std::string animFile = _sdf->GetValueString("filename");
    std::string extension = animFile.substr(animFile.rfind(".") + 1,
        animFile.size());
    double scale = _sdf->GetValueDouble("scale");
    Skeleton *skel = NULL;

    if (extension == "bvh")
    {
      BVHLoader loader;
      skel = loader.Load(animFile, scale);
    }
    else
      if (extension == "dae")
      {
        MeshManager::Instance()->Load(animFile);
        const Mesh *animMesh = NULL;
        if (MeshManager::Instance()->HasMesh(animFile))
          animMesh = MeshManager::Instance()->GetMesh(animFile);
        if (animMesh && animMesh->HasSkeleton())
        {
          skel = animMesh->GetSkeleton();
          skel->Scale(scale);
        }
      }

    if (!skel || skel->GetNumAnimations() == 0)
      gzerr << "Failed to load animation.";
    else
    {
      bool compatible = true;
      std::map<std::string, std::string> skelMap;
      if (this->skeleton->GetNumNodes() != skel->GetNumNodes())
        compatible = false;
      else
        for (unsigned int i = 0; i < this->skeleton->GetNumNodes(); i++)
        {
          SkeletonNode *skinNode = this->skeleton->GetNodeByHandle(i);
          SkeletonNode *animNode = skel->GetNodeByHandle(i);
          if (animNode->GetChildCount() != skinNode->GetChildCount())
          {
            compatible = false;
            break;
          }
          else
            skelMap[skinNode->GetName()] = animNode->GetName();
        }

      if (!compatible)
      {
        gzerr << "Skin and animation " << animName <<
              " skeletons are not compatible.\n";
      }
      else
      {
        this->skelAnimation[animName] =
            skel->GetAnimation(0);
        this->interpolateX[animName] = _sdf->GetValueBool("interpolate_x");
        this->skelNodesMap[animName] = skelMap;
      }
    }
  }
}

//////////////////////////////////////////////////
void Actor::Init()
{
  this->prevFrameTime = this->world->GetSimTime();
  if (this->autoStart)
    this->Play();
  this->mainLink = this->GetChildLink(this->GetName() + "_origin");
}

//////////////////////////////////////////////////
void Actor::Play()
{
  this->active = true;
  this->playStartTime = this->world->GetSimTime();
  this->lastScriptTime = std::numeric_limits<double>::max();
}

//////////////////////////////////////////////////
void Actor::Stop()
{
  this->active = false;
}

//////////////////////////////////////////////////
bool Actor::IsActive()
{
  return this->active;
}

///////////////////////////////////////////////////
void Actor::Update()
{
  if (!this->active)
    return;

  common::Time currentTime = this->world->GetSimTime();

  /// do not refresh animation more faster the 30 Hz sim time
  if ((currentTime - this->prevFrameTime).Double() < (1.0 / 30.0))
    return;

  double scriptTime = currentTime.Double() - this->startDelay -
            this->playStartTime.Double();

  /// waiting for delayed start
  if (scriptTime < 0)
    return;

  if (scriptTime >= this->scriptLength)
  {
    if (!this->loop)
      return;
    else
    {
      scriptTime = scriptTime - this->scriptLength;
      this->playStartTime = currentTime - scriptTime;
    }
  }

  /// at this point we are certain that a new frame will be animated
  this->prevFrameTime = currentTime;

  TrajectoryInfo tinfo;

  unsigned int tid = 0;

  for (unsigned int i = 0; i < this->trajInfo.size(); i++)
    if (this->trajInfo[i].startTime <= scriptTime &&
          this->trajInfo[i].endTime >= scriptTime)
    {
      tinfo = this->trajInfo[i];
      tid = i;
      break;
    }

  scriptTime = scriptTime - tinfo.startTime;

  SkeletonAnimation *skelAnim = this->skelAnimation[tinfo.type];
  std::map<std::string, std::string> skelMap = this->skelNodesMap[tinfo.type];

  common::PoseKeyFrame posFrame(0.0);
  this->trajectories[tid]->SetTime(scriptTime);
  this->trajectories[tid]->GetInterpolatedKeyFrame(posFrame);

  msgs::PoseAnimation msg;
  msg.set_model_name(this->visualName);

  std::map<std::string, math::Matrix4> frame;
  if (!this->interpolateX[tinfo.type])
    frame = skelAnim->GetPoseAt(scriptTime);
  else
  {
    common::PoseKeyFrame *frame0 = dynamic_cast<common::PoseKeyFrame*>
        (this->trajectories[tid]->GetKeyFrame(0));
    math::Vector3 framePos = posFrame.GetTranslation() -
        frame0->GetTranslation();
    frame = skelAnim->GetPoseAtX(framePos.GetLength(),
              skelMap[this->skeleton->GetRootNode()->GetName()]);
  }

  for (unsigned int i = 0; i < this->skeleton->GetNumNodes(); i++)
  {
    SkeletonNode *bone = this->skeleton->GetNodeByHandle(i);
    SkeletonNode *parentBone = bone->GetParent();
    math::Matrix4 transform(math::Matrix4::IDENTITY);
    if (frame.find(skelMap[bone->GetName()]) != frame.end())
      transform = frame[skelMap[bone->GetName()]];
    else
      transform = bone->GetTransform();

    LinkPtr currentLink = this->GetChildLink(bone->GetName());
    if (parentBone)
    {
      math::Pose bonePose;
      bonePose.pos = transform.GetTranslation();
      bonePose.rot = transform.GetRotation();
      if (!bonePose.IsFinite())
      {
        std::cerr << "ACTOR: " << currentTime << " " << bone->GetName()
                  << " " << bonePose << "\n";
        bonePose.Correct();
      }

      msgs::Pose *msg_pose = msg.add_pose();
      msg_pose->set_name(bone->GetName());
      msg_pose->mutable_position()->CopyFrom(msgs::Convert(bonePose.pos));
      msg_pose->mutable_orientation()->CopyFrom(msgs::Convert(bonePose.rot));

      LinkPtr parentLink = this->GetChildLink(
                              skelMap[parentBone->GetName()]);
      math::Pose parentPose = parentLink->GetWorldPose();
      math::Matrix4 parentTrans(math::Matrix4::IDENTITY);
      parentTrans = parentPose.rot.GetAsMatrix4();
      parentTrans.SetTranslate(parentPose.pos);
      transform = parentTrans * transform;
    }
    else
    {
      math::Pose modelPose;

      modelPose.pos = transform.GetTranslation();
      math::Vector3 yPos(0.0, modelPose.pos.y, 0.0);
      math::Vector3 zPos(0.0, 0.0, modelPose.pos.z);

      modelPose.pos = posFrame.GetTranslation() +
                        posFrame.GetRotation().RotateVector(yPos);
      modelPose.rot = posFrame.GetRotation();

      this->mainLink->SetWorldPose(modelPose);

      math::Matrix4 modelTrans(modelPose.rot.GetAsMatrix4());
      modelTrans.SetTranslate(modelPose.pos + zPos);
      transform.SetTranslate(math::Vector3(0, 0, 0));
      transform = modelTrans * transform;

      this->lastScriptTime = scriptTime;
    }
    currentLink->SetWorldPose(transform.GetAsPose());
  }

  msgs::Time *stamp = msg.add_time();
  stamp->CopyFrom(msgs::Convert(currentTime));

  if (this->bonePosePub && this->bonePosePub->HasConnections())
    this->bonePosePub->Publish(msg);
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
  meshSdf->GetAttribute("scale")->Set(math::Vector3(this->skinScale,
      this->skinScale, this->skinScale));

  /// use a material with shading for now
  sdf::ElementPtr matSdf = visualSdf->GetOrCreateElement("material");
  matSdf->GetAttribute("script")->Set("Gazebo/Blue");
  sdf::ElementPtr colorSdf = matSdf->GetOrCreateElement("ambient");
  colorSdf->GetAttribute("rgba")->Set(Color::Blue);
}
