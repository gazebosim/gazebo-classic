/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <sstream>
#include <limits>
#include <algorithm>

#include "gazebo/msgs/msgs.hh"

#include "gazebo/common/KeyFrame.hh"
#include "gazebo/common/Animation.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/CommonTypes.hh"
#include "gazebo/common/MeshManager.hh"
#include "gazebo/common/Mesh.hh"
#include "gazebo/common/Skeleton.hh"
#include "gazebo/common/SkeletonAnimation.hh"
#include "gazebo/common/BVHLoader.hh"

#include "gazebo/physics/World.hh"
#include "gazebo/physics/Joint.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/ActorPrivate.hh"
#include "gazebo/physics/Actor.hh"
#include "gazebo/physics/PhysicsIface.hh"

#include "gazebo/transport/Node.hh"

using namespace gazebo;
using namespace physics;
using namespace common;

//////////////////////////////////////////////////
Actor::Actor(BasePtr _parent)
: Model(*new ActorPrivate, _parent),
  actorDPtr(static_cast<ActorPrivate*>(this->modelDPtr))
{
  this->AddType(ACTOR);
  this->actorDPtr->skeleton = NULL;
  this->actorDPtr->pathLength = 0.0;
  this->actorDPtr->lastTraj = 1e+5;
}

//////////////////////////////////////////////////
Actor::~Actor()
{
  this->actorDPtr->skelAnimation.clear();
  this->actorDPtr->bonePosePub.reset();
}

//////////////////////////////////////////////////
void Actor::Load(sdf::ElementPtr _sdf)
{
  sdf::ElementPtr skinSdf = _sdf->GetElement("skin");
  this->actorDPtr->skinFile = skinSdf->Get<std::string>("filename");
  this->actorDPtr->skinScale = skinSdf->Get<double>("scale");

  MeshManager::Instance()->Load(this->actorDPtr->skinFile);
  std::string actorName = _sdf->Get<std::string>("name");

/*  double radius = 1.0;
  unsigned int pointNum = 32;
  for (unsigned int i = 0; i < pointNum; i++)
  {
    double angle = (2 * i * M_PI) / pointNum;
    double x = radius * sin(angle);
    double y = radius * cos(angle);
    if (ignition::math::equal(x, 0.0))
      x = 0;
    if (ignition::math::equal(y, 0.0))
      y = 0;
    std::cerr << x << " " << y << " 0 0 0 " << angle << "\n";
  }   */

  if (MeshManager::Instance()->HasMesh(this->actorDPtr->skinFile))
  {
    this->actorDPtr->mesh =
      MeshManager::Instance()->GetMesh(this->actorDPtr->skinFile);
    if (!this->actorDPtr->mesh->HasSkeleton())
      gzthrow("Collada file does not contain skeletal animation.");
    this->actorDPtr->skeleton = this->actorDPtr->mesh->GetSkeleton();
    this->actorDPtr->skeleton->Scale(this->actorDPtr->skinScale);
    /// create the link sdfs for the model
    NodeMap nodes = this->actorDPtr->skeleton->GetNodes();

    /// self_collide should be added to prevent error messages
    // _sdf->GetElement("self_collide")->Set(false);

    sdf::ElementPtr linkSdf;
    linkSdf = _sdf->GetElement("link");
    linkSdf->GetAttribute("name")->Set(actorName + "_pose");
    linkSdf->GetElement("gravity")->Set(false);
    linkSdf->GetElement("self_collide")->Set(false);
    sdf::ElementPtr linkPose = linkSdf->GetElement("pose");

    // this->AddSphereInertia(linkSdf, ignition::math::Pose3d(), 1.0, 0.01);
    // this->AddSphereCollision(linkSdf, actorName + "_pose_col",
    //                          ignition::math::Pose3d(), 0.02);
    // this->AddBoxVisual(linkSdf, actorName + "_pose_vis",
    // ignition::math::Pose3d(),
    // ignition::math::Vector3d(0.05, 0.05, 0.05), "Gazebo/White",
    // Color::White);
    this->AddActorVisual(linkSdf, actorName + "_visual",
        ignition::math::Pose3d::Zero);
    std::string actorLinkName = actorName + "::" + actorName + "_pose";
    this->actorDPtr->visualName = actorLinkName + "::"
                             + actorName + "_visual";

    for (NodeMapIter iter = nodes.begin(); iter != nodes.end(); ++iter)
    {
      SkeletonNode *bone = iter->second;

      linkSdf = _sdf->AddElement("link");

      linkSdf->GetAttribute("name")->Set(bone->GetName());
      linkSdf->GetElement("gravity")->Set(false);
      linkSdf->GetElement("self_collide")->Set(false);
      linkPose = linkSdf->GetElement("pose");
      ignition::math::Pose3d pose(bone->ModelTransform().Translation(),
                                  bone->ModelTransform().Rotation());
      if (bone->IsRootNode())
        pose = ignition::math::Pose3d::Zero;
      linkPose->Set(pose);

      /// FIXME hardcoded inertia of a sphere with mass 1.0 and radius 0.01
      this->AddSphereInertia(linkSdf, ignition::math::Pose3d::Zero, 1.0, 0.01);

      /// FIXME hardcoded collision to sphere with radius 0.02
      this->AddSphereCollision(linkSdf, bone->GetName() + "_collision",
                       ignition::math::Pose3d::Zero, 0.02);

      /// FIXME hardcoded visual to red sphere with radius 0.02
      if (bone->IsRootNode())
      {
        this->AddSphereVisual(linkSdf, bone->GetName() + "__SKELETON_VISUAL__",
            ignition::math::Pose3d::Zero, 0.02, "Gazebo/Blue", Color::Blue);
      }
      else
        if (bone->GetChildCount() == 0)
        {
          this->AddSphereVisual(linkSdf, bone->GetName() +
              "__SKELETON_VISUAL__", ignition::math::Pose3d::Zero, 0.02,
              "Gazebo/Yellow", Color::Yellow);
        }
        else
        {
          this->AddSphereVisual(linkSdf, bone->GetName() +
              "__SKELETON_VISUAL__", ignition::math::Pose3d::Zero, 0.02,
              "Gazebo/Red", Color::Red);
        }

      for (unsigned int i = 0; i < bone->GetChildCount(); ++i)
      {
        SkeletonNode *curChild = bone->GetChild(i);

        ignition::math::Vector3d dir =
          curChild->ModelTransform().Translation() -
          bone->ModelTransform().Translation();
        double length = dir.Length();

        if (!ignition::math::equal(length, 0.0))
        {
          ignition::math::Vector3d r =
            curChild->Transform().Translation();
          ignition::math::Vector3d linkPos =
            ignition::math::Vector3d(r.X() / 2.0, r.Y() / 2.0, r.Z() / 2.0);
          double theta = atan2(dir.Y(), dir.X());
          double phi = acos(dir.Z() / length);

          ignition::math::Pose3d bonePose(linkPos,
              ignition::math::Quaterniond(0.0, phi, theta));
          bonePose.Rot() = pose.Rot().Inverse() * bonePose.Rot();

          this->AddBoxVisual(linkSdf, bone->GetName() + "_" +
            curChild->GetName() + "__SKELETON_VISUAL__", bonePose,
            ignition::math::Vector3d(0.02, 0.02, length),
            "Gazebo/Green", Color::Green);
        }
      }
    }

    sdf::ElementPtr animSdf = _sdf->GetElement("animation");

    while (animSdf)
    {
      this->LoadAnimation(animSdf);
      animSdf = animSdf->GetNextElement("animation");
    }

    this->LoadScript(_sdf->GetElement("script"));

    /// we are ready to load the links
    Model::Load(_sdf);
    LinkPtr actorLinkPtr(Model::LinkByName(actorLinkName));
    if (actorLinkPtr)
    {
       msgs::Visual actorVisualMsg = actorLinkPtr->VisualMessage(
         this->actorDPtr->visualName);
       if (actorVisualMsg.has_id())
         this->actorDPtr->visualId = actorVisualMsg.id();
       else
         gzerr << "No actor visual message found.";
    }
    else
    {
      gzerr << "No actor link found.";
    }
    this->actorDPtr->bonePosePub =
      this->actorDPtr->node->Advertise<msgs::PoseAnimation>(
          "~/skeleton_pose/info", 10);
  }
}

//////////////////////////////////////////////////
void Actor::LoadScript(sdf::ElementPtr _sdf)
{
  this->actorDPtr->loop = _sdf->Get<bool>("loop");
  this->actorDPtr->startDelay = _sdf->Get<double>("delay_start");
  this->actorDPtr->autoStart = _sdf->Get<bool>("auto_start");
  this->actorDPtr->active = this->actorDPtr->autoStart;

  if (_sdf->HasElement("trajectory"))
  {
    sdf::ElementPtr trajSdf = _sdf->GetElement("trajectory");
    while (trajSdf)
    {
      if (this->actorDPtr->skelAnimation.find(
            trajSdf->Get<std::string>("type")) ==
          this->actorDPtr->skelAnimation.end())
      {
        gzwarn << "Resource not found for trajectory of type " <<
                  trajSdf->Get<std::string>("type") << "\n";
        continue;
      }

      TrajectoryInfo tinfo;
      tinfo.id = trajSdf->Get<int>("id");
      tinfo.type = trajSdf->Get<std::string>("type");
      std::vector<TrajectoryInfo>::iterator iter =
        this->actorDPtr->trajInfo.begin();
      while (iter != this->actorDPtr->trajInfo.end())
      {
        if (iter->id > tinfo.id)
          break;
        ++iter;
      }

      unsigned int idx = iter - this->actorDPtr->trajInfo.begin();
      this->actorDPtr->trajInfo.insert(iter, tinfo);

      std::map<double, ignition::math::Pose3d> points;

      if (trajSdf->HasElement("waypoint"))
      {
        sdf::ElementPtr wayptSdf = trajSdf->GetElement("waypoint");
        while (wayptSdf)
        {
          points[wayptSdf->Get<double>("time")] =
            wayptSdf->Get<ignition::math::Pose3d>("pose");
          wayptSdf = wayptSdf->GetNextElement("waypoint");
        }

        std::map<double, ignition::math::Pose3d>::reverse_iterator last =
          points.rbegin();
        std::stringstream animName;
        animName << tinfo.type << "_" << tinfo.id;
        common::PoseAnimation *anim = new common::PoseAnimation(animName.str(),
                                                          last->first, false);
        this->actorDPtr->trajInfo[idx].duration = last->first;
        this->actorDPtr->trajInfo[idx].translated = true;

        for (std::map<double, ignition::math::Pose3d>::iterator pIter =
            points.begin(); pIter != points.end(); ++pIter)
        {
          common::PoseKeyFrame *key;
          if (pIter == points.begin() &&
              !ignition::math::equal(pIter->first, 0.0))
          {
            key = anim->CreateKeyFrame(0.0);
            key->Translation(pIter->second.Pos());
            key->Rotation(pIter->second.Rot());
          }
          key = anim->CreateKeyFrame(pIter->first);
          key->Translation(pIter->second.Pos());
          key->Rotation(pIter->second.Rot());
        }

        this->actorDPtr->trajectories[this->actorDPtr->trajInfo[idx].id] = anim;
      }
      else
      {
        this->actorDPtr->trajInfo[idx].duration =
                this->actorDPtr->skelAnimation[
                this->actorDPtr->trajInfo[idx].type]->GetLength();
        this->actorDPtr->trajInfo[idx].translated = false;
      }

      trajSdf = trajSdf->GetNextElement("trajectory");
    }
  }
  double scriptTime = 0.0;
  if (!this->actorDPtr->skelAnimation.empty())
  {
    if (this->actorDPtr->trajInfo.empty())
    {
      TrajectoryInfo tinfo;
      tinfo.id = 0;
      tinfo.type = this->actorDPtr->skinFile;
      tinfo.startTime = 0.0;
      tinfo.duration =
        this->actorDPtr->skelAnimation.begin()->second->GetLength();
      tinfo.endTime = tinfo.duration;
      tinfo.translated = false;
      this->actorDPtr->trajInfo.push_back(tinfo);
      this->actorDPtr->interpolateX[this->actorDPtr->skinFile] = false;
    }
    for (unsigned int i = 0; i < this->actorDPtr->trajInfo.size(); i++)
    {
      this->actorDPtr->trajInfo[i].startTime = scriptTime;
      scriptTime += this->actorDPtr->trajInfo[i].duration;
      this->actorDPtr->trajInfo[i].endTime = scriptTime;
    }
  }
  this->actorDPtr->scriptLength = scriptTime;
}

//////////////////////////////////////////////////
void Actor::LoadAnimation(sdf::ElementPtr _sdf)
{
  std::string animName = _sdf->Get<std::string>("name");

  if (animName == "__default__")
  {
    this->actorDPtr->skelAnimation[this->actorDPtr->skinFile] =
        this->actorDPtr->skeleton->GetAnimation(0);
    std::map<std::string, std::string> skelMap;
    for (unsigned int i = 0; i < this->actorDPtr->skeleton->GetNumNodes(); ++i)
      skelMap[this->actorDPtr->skeleton->GetNodeByHandle(i)->GetName()] =
        this->actorDPtr->skeleton->GetNodeByHandle(i)->GetName();
    this->actorDPtr->skelNodesMap[this->actorDPtr->skinFile] = skelMap;
    this->actorDPtr->interpolateX[this->actorDPtr->skinFile] = false;
  }
  else
  {
    std::string animFile = _sdf->Get<std::string>("filename");
    std::string extension = animFile.substr(animFile.rfind(".") + 1,
        animFile.size());
    double animScale = _sdf->Get<double>("scale");
    Skeleton *skel = NULL;

    if (extension == "bvh")
    {
      BVHLoader loader;
      skel = loader.Load(animFile, animScale);
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
          skel->Scale(animScale);
        }
      }

    if (!skel || skel->GetNumAnimations() == 0)
      gzerr << "Failed to load animation.";
    else
    {
      bool compatible = true;
      std::map<std::string, std::string> skelMap;
      if (this->actorDPtr->skeleton->GetNumNodes() != skel->GetNumNodes())
        compatible = false;
      else
        for (unsigned int i = 0;
            i < this->actorDPtr->skeleton->GetNumNodes(); ++i)
        {
          SkeletonNode *skinNode =
            this->actorDPtr->skeleton->GetNodeByHandle(i);
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
        this->actorDPtr->skelAnimation[animName] =
            skel->GetAnimation(0);
        this->actorDPtr->interpolateX[animName] =
          _sdf->Get<bool>("interpolate_x");
        this->actorDPtr->skelNodesMap[animName] = skelMap;
      }
    }
  }
}

//////////////////////////////////////////////////
void Actor::Init()
{
  this->actorDPtr->prevFrameTime = this->actorDPtr->world->SimTime();
  if (this->actorDPtr->autoStart)
    this->Play();
  this->actorDPtr->mainLink = this->ChildLink(this->Name() + "_pose");
}

//////////////////////////////////////////////////
void Actor::Play()
{
  this->actorDPtr->active = true;
  this->actorDPtr->playStartTime = this->actorDPtr->world->SimTime();
  this->actorDPtr->lastScriptTime = std::numeric_limits<double>::max();
}

//////////////////////////////////////////////////
void Actor::Stop()
{
  this->actorDPtr->active = false;
}

//////////////////////////////////////////////////
bool Actor::IsActive() const
{
  return this->actorDPtr->active;
}

///////////////////////////////////////////////////
void Actor::Update()
{
  if (!this->actorDPtr->active)
    return;

  common::Time currentTime = this->actorDPtr->world->SimTime();

  /// do not refresh animation more faster the 30 Hz sim time
  /// TODO: Reducing to 20 Hz. Because there were memory corruption
  /// and segmentation faults. Possibly due to some dangling pointers
  /// in pose message processing. This will need a proper fix. Just a
  /// workaround for now.
  if ((currentTime - this->actorDPtr->prevFrameTime).Double() < (1.0 / 20.0))
    return;

  double scriptTime = currentTime.Double() - this->actorDPtr->startDelay -
            this->actorDPtr->playStartTime.Double();

  /// waiting for delayed start
  if (scriptTime < 0)
    return;

  if (scriptTime >= this->actorDPtr->scriptLength)
  {
    if (!this->actorDPtr->loop)
      return;
    else
    {
      scriptTime = scriptTime - this->actorDPtr->scriptLength;
      this->actorDPtr->playStartTime = currentTime - scriptTime;
    }
  }

  /// at this point we are certain that a new frame will be animated
  this->actorDPtr->prevFrameTime = currentTime;

  TrajectoryInfo tinfo;

  for (unsigned int i = 0; i < this->actorDPtr->trajInfo.size(); ++i)
    if (this->actorDPtr->trajInfo[i].startTime <= scriptTime &&
          this->actorDPtr->trajInfo[i].endTime >= scriptTime)
    {
      tinfo = this->actorDPtr->trajInfo[i];
      break;
    }

  scriptTime = scriptTime - tinfo.startTime;

  SkeletonAnimation *skelAnim = this->actorDPtr->skelAnimation[tinfo.type];
  std::map<std::string, std::string> skelMap =
    this->actorDPtr->skelNodesMap[tinfo.type];

  ignition::math::Pose3d modelPose;
  std::map<std::string, ignition::math::Matrix4d> frame;
  if (this->actorDPtr->trajectories.find(tinfo.id) !=
      this->actorDPtr->trajectories.end())
  {
    common::PoseKeyFrame posFrame(0.0);
    this->actorDPtr->trajectories[tinfo.id]->SetTime(scriptTime);
    this->actorDPtr->trajectories[tinfo.id]->GetInterpolatedKeyFrame(posFrame);

    modelPose.Pos() = posFrame.Translation();
    modelPose.Rot() = posFrame.Rotation();

    if (this->actorDPtr->lastTraj == tinfo.id)
    {
      this->actorDPtr->pathLength +=
        fabs(this->actorDPtr->lastPos.Distance(modelPose.Pos()));
    }
    else
    {
      common::PoseKeyFrame *frame0 = dynamic_cast<common::PoseKeyFrame*>
        (this->actorDPtr->trajectories[tinfo.id]->GetKeyFrame(0));
      ignition::math::Vector3d vector3Ign;
      vector3Ign = frame0->Translation();
      this->actorDPtr->pathLength = fabs(modelPose.Pos().Distance(vector3Ign));
    }
    this->actorDPtr->lastPos = modelPose.Pos();
  }

  if (this->actorDPtr->interpolateX[tinfo.type] &&
      this->actorDPtr->trajectories.find(tinfo.id) !=
      this->actorDPtr->trajectories.end())
  {
    frame = skelAnim->PoseAtX(this->actorDPtr->pathLength,
              skelMap[this->actorDPtr->skeleton->GetRootNode()->GetName()]);
  }
  else
  {
    frame = skelAnim->PoseAt(scriptTime);
  }

  this->actorDPtr->lastTraj = tinfo.id;

  ignition::math::Matrix4d rootTrans =
    frame[skelMap[this->actorDPtr->skeleton->GetRootNode()->GetName()]];

  ignition::math::Vector3d rootPos = rootTrans.Translation();
  ignition::math::Quaterniond rootRot = rootTrans.Rotation();

  if (tinfo.translated)
    rootPos.X() = 0.0;
  ignition::math::Pose3d actorPose;
  actorPose.Pos() = modelPose.Pos() + modelPose.Rot().RotateVector(rootPos);
  actorPose.Rot() = modelPose.Rot() * rootRot;

  ignition::math::Matrix4d rootM(actorPose.Rot());
  rootM.Translate(actorPose.Pos());

  frame[skelMap[this->actorDPtr->skeleton->GetRootNode()->GetName()]] = rootM;

  this->SetPose(frame, skelMap, currentTime.Double());

  this->actorDPtr->lastScriptTime = scriptTime;
}

//////////////////////////////////////////////////
void Actor::SetPose(
    const std::map<std::string, ignition::math::Matrix4d> &_frame,
    const std::map<std::string, std::string> &_skelMap, const double _time)
{
  msgs::PoseAnimation msg;
  msg.set_model_name(this->actorDPtr->visualName);
  msg.set_model_id(this->actorDPtr->visualId);

  ignition::math::Matrix4d modelTrans(ignition::math::Matrix4d::Identity);
  ignition::math::Pose3d mainLinkPose;

  for (unsigned int i = 0; i < this->actorDPtr->skeleton->GetNumNodes(); ++i)
  {
    SkeletonNode *bone = this->actorDPtr->skeleton->GetNodeByHandle(i);
    SkeletonNode *parentBone = bone->GetParent();
    ignition::math::Matrix4d transform(ignition::math::Matrix4d::Identity);
    std::map<std::string, std::string>::const_iterator skelIter =
      _skelMap.find(bone->GetName());

    if (skelIter == _skelMap.end())
      continue;

    std::map<std::string, ignition::math::Matrix4d>::const_iterator frameIter =
      _frame.find(skelIter->second);

    if (frameIter != _frame.end())
      transform = frameIter->second;
    else
      transform = bone->Transform();

    LinkPtr currentLink = this->ChildLink(bone->GetName());
    ignition::math::Pose3d bonePose = transform.Pose();

    if (!bonePose.IsFinite())
    {
      std::cerr << "ACTOR: " << _time << " " << bone->GetName()
                << " " << bonePose << "\n";
      bonePose.Correct();
    }

    msgs::Pose *bone_pose = msg.add_pose();
    bone_pose->set_name(bone->GetName());

    if (!parentBone)
    {
      bone_pose->mutable_position()->CopyFrom(
          msgs::Convert(ignition::math::Vector3d()));
      bone_pose->mutable_orientation()->CopyFrom(msgs::Convert(
            ignition::math::Quaterniond()));
      mainLinkPose = bonePose;
    }
    else
    {
      bone_pose->mutable_position()->CopyFrom(msgs::Convert(bonePose.Pos()));
      bone_pose->mutable_orientation()->CopyFrom(msgs::Convert(bonePose.Rot()));
      LinkPtr parentLink = this->ChildLink(parentBone->GetName());
      ignition::math::Pose3d parentPose = parentLink->WorldPose();
      ignition::math::Matrix4d parentTrans(parentPose.Rot());
      parentTrans.Translate(parentPose.Pos());
      transform = (parentTrans * transform);
    }

    msgs::Pose *link_pose = msg.add_pose();
    link_pose->set_name(currentLink->ScopedName());
    link_pose->set_id(currentLink->Id());
    ignition::math::Pose3d linkPose = transform.Pose() - mainLinkPose;
    link_pose->mutable_position()->CopyFrom(msgs::Convert(linkPose.Pos()));
    link_pose->mutable_orientation()->CopyFrom(msgs::Convert(linkPose.Rot()));
    currentLink->SetWorldPose(transform.Pose(), true, false);
  }

  msgs::Time *stamp = msg.add_time();
  stamp->CopyFrom(msgs::Convert(common::Time(_time)));

  msgs::Pose *model_pose = msg.add_pose();
  model_pose->set_name(this->ScopedName());
  model_pose->set_id(this->Id());
  model_pose->mutable_position()->CopyFrom(msgs::Convert(mainLinkPose.Pos()));
  model_pose->mutable_orientation()->CopyFrom(
      msgs::Convert(mainLinkPose.Rot()));

  if (this->actorDPtr->bonePosePub &&
      this->actorDPtr->bonePosePub->HasConnections())
  {
    this->actorDPtr->bonePosePub->Publish(msg);
  }
  this->SetWorldPose(mainLinkPose, true, false);
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
void Actor::AddSphereInertia(const sdf::ElementPtr &_linkSdf,
                             const ignition::math::Pose3d &_pose,
                             const double _mass, const double _radius)
{
  double ixx = 2.0 * _mass * _radius * _radius / 5.0;
  sdf::ElementPtr inertialSdf = _linkSdf->GetElement("inertial");
  sdf::ElementPtr inertialPoseSdf = inertialSdf->GetElement("pose");
  inertialPoseSdf->Set(_pose);
  inertialSdf->GetElement("mass")->Set(_mass);
  sdf::ElementPtr tensorSdf = inertialSdf->GetElement("inertia");
  tensorSdf->GetElement("ixx")->Set(ixx);
  tensorSdf->GetElement("ixy")->Set(0.00);
  tensorSdf->GetElement("ixz")->Set(0.00);
  tensorSdf->GetElement("iyy")->Set(ixx);
  tensorSdf->GetElement("iyz")->Set(0.00);
  tensorSdf->GetElement("izz")->Set(ixx);
}

//////////////////////////////////////////////////
void Actor::AddSphereCollision(const sdf::ElementPtr &_linkSdf,
                               const std::string &_name,
                               const ignition::math::Pose3d &_pose,
                               const double _radius)
{
  sdf::ElementPtr collisionSdf = _linkSdf->GetElement("collision");
  collisionSdf->GetAttribute("name")->Set(_name);
  sdf::ElementPtr collPoseSdf = collisionSdf->GetElement("pose");
  collPoseSdf->Set(_pose);
  sdf::ElementPtr geomColSdf = collisionSdf->GetElement("geometry");
  sdf::ElementPtr sphereColSdf = geomColSdf->GetElement("sphere");
  sphereColSdf->GetElement("radius")->Set(_radius);
}

//////////////////////////////////////////////////
void Actor::AddSphereVisual(const sdf::ElementPtr &_linkSdf,
    const std::string &_name, const ignition::math::Pose3d &_pose,
    const double _radius, const std::string &_material,
    const common::Color &_ambient)
{
  sdf::ElementPtr visualSdf = _linkSdf->GetElement("visual");
  visualSdf->GetAttribute("name")->Set(_name);
  sdf::ElementPtr visualPoseSdf = visualSdf->GetElement("pose");
  visualPoseSdf->Set(_pose);
  sdf::ElementPtr geomVisSdf = visualSdf->GetElement("geometry");
  sdf::ElementPtr sphereVisSdf = geomVisSdf->GetElement("sphere");
  sphereVisSdf->GetElement("radius")->Set(_radius);
  sdf::ElementPtr matSdf = visualSdf->GetElement("material");
  matSdf->GetElement("script")->Set(_material);
  sdf::ElementPtr colorSdf = matSdf->GetElement("ambient");
  colorSdf->Set(_ambient);
}

//////////////////////////////////////////////////
void Actor::AddBoxVisual(const sdf::ElementPtr &_linkSdf,
    const std::string &_name, const ignition::math::Pose3d &_pose,
    const ignition::math::Vector3d &_size, const std::string &_material,
    const common::Color &_ambient)
{
  sdf::ElementPtr visualSdf = _linkSdf->AddElement("visual");
  visualSdf->GetAttribute("name")->Set(_name);
  sdf::ElementPtr visualPoseSdf = visualSdf->GetElement("pose");
  visualPoseSdf->Set(_pose);
  sdf::ElementPtr geomVisSdf = visualSdf->GetElement("geometry");
  sdf::ElementPtr boxSdf = geomVisSdf->GetElement("box");
  boxSdf->GetElement("size")->Set(_size);
  sdf::ElementPtr matSdf = visualSdf->GetElement("material");
  matSdf->GetElement("script")->Set(_material);
  sdf::ElementPtr colorSdf = matSdf->GetElement("ambient");
  colorSdf->Set(_ambient);
}

//////////////////////////////////////////////////
void Actor::AddActorVisual(const sdf::ElementPtr &_linkSdf,
    const std::string &_name, const ignition::math::Pose3d &_pose)
{
  sdf::ElementPtr visualSdf = _linkSdf->AddElement("visual");
  visualSdf->GetAttribute("name")->Set(_name);
  sdf::ElementPtr visualPoseSdf = visualSdf->GetElement("pose");
  visualPoseSdf->Set(_pose);
  sdf::ElementPtr geomVisSdf = visualSdf->GetElement("geometry");
  sdf::ElementPtr meshSdf = geomVisSdf->GetElement("mesh");
  meshSdf->GetElement("uri")->Set(this->actorDPtr->skinFile);
  meshSdf->GetElement("scale")->Set(math::Vector3(this->actorDPtr->skinScale,
      this->actorDPtr->skinScale, this->actorDPtr->skinScale));
}

//////////////////////////////////////////////////
TrajectoryInfo::TrajectoryInfo()
  : id(0), type(""), duration(0.0), startTime(0.0), endTime(0.0),
  translated(false)
{
}
