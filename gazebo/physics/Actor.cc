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

#include "gazebo/common/BVHLoader.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/KeyFrame.hh"
#include "gazebo/common/MeshManager.hh"
#include "gazebo/common/Mesh.hh"
#include "gazebo/common/Skeleton.hh"
#include "gazebo/common/SkeletonAnimation.hh"

#include "gazebo/msgs/msgs.hh"

#include "gazebo/physics/Actor.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/World.hh"

#include "gazebo/transport/Node.hh"

using namespace gazebo;
using namespace physics;
using namespace common;

//////////////////////////////////////////////////
Actor::Actor(BasePtr _parent)
  : Model(_parent)
{
  this->AddType(ACTOR);
  this->pathLength = 0.0;
  this->lastTraj = 1e+5;
  this->skinScale = 1.0;
}

//////////////////////////////////////////////////
Actor::~Actor()
{
  this->bonePosePub.reset();
  this->customTrajectoryInfo.reset();

  this->skelAnimation.clear();
  this->skelNodesMap.clear();
  this->interpolateX.clear();
  this->trajInfo.clear();
  this->trajectories.clear();

  this->mainLink.reset();

  // mesh and skeleton should be deleted by the MeshManager
}

//////////////////////////////////////////////////
void Actor::Load(sdf::ElementPtr _sdf)
{
  sdf::ElementPtr skinSdf = _sdf->GetElement("skin");
  this->skinFile = skinSdf->Get<std::string>("filename");
  this->skinScale = skinSdf->Get<double>("scale");

  MeshManager::Instance()->Load(this->skinFile);
  std::string actorName = _sdf->Get<std::string>("name");

  if (MeshManager::Instance()->HasMesh(this->skinFile))
  {
    this->mesh = MeshManager::Instance()->GetMesh(this->skinFile);
    if (!this->mesh->HasSkeleton())
      gzthrow("Collada file does not contain skeletal animation.");
    this->skeleton = mesh->GetSkeleton();
    this->skeleton->Scale(this->skinScale);
    /// create the link sdfs for the model
    NodeMap nodes = this->skeleton->GetNodes();

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
    this->visualName = actorLinkName + "::"
                             + actorName + "_visual";

    for (NodeMapIter iter = nodes.begin(); iter != nodes.end(); ++iter)
    {
      SkeletonNode* bone = iter->second;

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
    LinkPtr actorLinkPtr = Model::GetLink(actorLinkName);
    if (actorLinkPtr)
    {
       msgs::Visual actorVisualMsg = actorLinkPtr->GetVisualMessage(
         this->visualName);
       if (actorVisualMsg.has_id())
         this->visualId = actorVisualMsg.id();
       else
         gzerr << "No actor visual message found." << std::endl;
    }
    else
    {
      gzerr << "No actor link found." << std::endl;
    }

    // Advertise skeleton pose info
    this->bonePosePub = this->node->Advertise<msgs::PoseAnimation>(
                                       "~/skeleton_pose/info", 10);
  }
}

//////////////////////////////////////////////////
void Actor::LoadScript(sdf::ElementPtr _sdf)
{
  this->loop = _sdf->Get<bool>("loop");
  this->startDelay = _sdf->Get<double>("delay_start");
  this->autoStart = _sdf->Get<bool>("auto_start");
  this->active = this->autoStart;

  if (_sdf->HasElement("trajectory"))
  {
    sdf::ElementPtr trajSdf = _sdf->GetElement("trajectory");
    while (trajSdf)
    {
      if (this->skelAnimation.find(trajSdf->Get<std::string>("type")) ==
              this->skelAnimation.end())
      {
        gzwarn << "Resource not found for trajectory of type [" <<
                  trajSdf->Get<std::string>("type") << "]" << std::endl;
        continue;
      }

      TrajectoryInfo tinfo;
      tinfo.id = trajSdf->Get<int>("id");
      tinfo.type = trajSdf->Get<std::string>("type");
      std::vector<TrajectoryInfo>::iterator iter = this->trajInfo.begin();
      while (iter != this->trajInfo.end())
      {
        if (iter->id > tinfo.id)
          break;
        ++iter;
      }

      unsigned int idx = iter - this->trajInfo.begin();
      this->trajInfo.insert(iter, tinfo);

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
        this->trajInfo[idx].duration = last->first;
        this->trajInfo[idx].translated = true;

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

        this->trajectories[this->trajInfo[idx].id] = anim;
      }
      else
      {
        this->trajInfo[idx].duration =
                this->skelAnimation[this->trajInfo[idx].type]->GetLength();
        this->trajInfo[idx].translated = false;
      }

      trajSdf = trajSdf->GetNextElement("trajectory");
    }
  }
  double time = 0.0;
  if (!this->skelAnimation.empty() && this->skelAnimation.begin()->second)
  {
    if (this->trajInfo.empty())
    {
      TrajectoryInfo tinfo;
      tinfo.id = 0;
      tinfo.type = this->skinFile;
      tinfo.startTime = 0.0;
      tinfo.duration = this->skelAnimation.begin()->second->GetLength();
      tinfo.endTime = tinfo.duration;
      tinfo.translated = false;
      this->trajInfo.push_back(tinfo);
      this->interpolateX[this->skinFile] = false;
    }
    for (unsigned int i = 0; i < this->trajInfo.size(); i++)
    {
      this->trajInfo[i].startTime = time;
      time += this->trajInfo[i].duration;
      this->trajInfo[i].endTime = time;
    }
  }
  this->scriptLength = time;
}

//////////////////////////////////////////////////
void Actor::LoadAnimation(sdf::ElementPtr _sdf)
{
  std::string animName = _sdf->Get<std::string>("name");

  if (animName == "__default__")
  {
    this->skelAnimation[this->skinFile] =
        this->skeleton->GetAnimation(0);
    std::map<std::string, std::string> skelMap;
    for (unsigned int i = 0; i < this->skeleton->GetNumNodes(); ++i)
      skelMap[this->skeleton->GetNodeByHandle(i)->GetName()] =
        this->skeleton->GetNodeByHandle(i)->GetName();
    this->skelNodesMap[this->skinFile] = skelMap;
    this->interpolateX[this->skinFile] = false;
  }
  else
  {
    std::string animFile = _sdf->Get<std::string>("filename");
    std::string extension = animFile.substr(animFile.rfind(".") + 1,
        animFile.size());
    double animScale = _sdf->Get<double>("scale");
    Skeleton *skel = nullptr;

    if (extension == "bvh")
    {
      BVHLoader loader;
      skel = loader.Load(animFile, animScale);
    }
    else
      if (extension == "dae")
      {
        MeshManager::Instance()->Load(animFile);
        const Mesh *animMesh = nullptr;
        if (MeshManager::Instance()->HasMesh(animFile))
          animMesh = MeshManager::Instance()->GetMesh(animFile);
        if (animMesh && animMesh->HasSkeleton())
        {
          skel = animMesh->GetSkeleton();
          skel->Scale(animScale);
        }
      }

    if (!skel || skel->GetNumAnimations() == 0)
      gzerr << "Failed to load animation." << std::endl;
    else
    {
      bool compatible = true;
      std::map<std::string, std::string> skelMap;
      if (this->skeleton->GetNumNodes() != skel->GetNumNodes())
        compatible = false;
      else
        for (unsigned int i = 0; i < this->skeleton->GetNumNodes(); ++i)
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
              " skeletons are not compatible." << std::endl;
      }
      else
      {
        this->skelAnimation[animName] =
            skel->GetAnimation(0);
        this->interpolateX[animName] = _sdf->Get<bool>("interpolate_x");
        this->skelNodesMap[animName] = skelMap;
      }
    }
  }
}

//////////////////////////////////////////////////
void Actor::Init()
{
  this->scriptTime = 0;
  this->prevFrameTime = this->world->GetSimTime();
  if (this->autoStart)
    this->Play();
  this->mainLink = this->GetChildLink(this->GetName() + "_pose");
}

//////////////////////////////////////////////////
void Actor::Play()
{
  this->active = true;
  this->playStartTime = this->world->GetSimTime();
}

//////////////////////////////////////////////////
void Actor::Stop()
{
  this->active = false;
}

//////////////////////////////////////////////////
bool Actor::IsActive() const
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
  /// TODO: Reducing to 20 Hz. Because there were memory corruption
  /// and segmentation faults. Possibly due to some dangling pointers
  /// in pose message processing. This will need a proper fix. Just a
  /// workaround for now.
  if ((currentTime - this->prevFrameTime).Double() < (1.0 / 20.0))
    return;

  TrajectoryInfo *tinfo = nullptr;

  if (!this->customTrajectoryInfo)
  {
    this->scriptTime = currentTime.Double() - this->startDelay -
              this->playStartTime.Double();

    /// waiting for delayed start
    if (this->scriptTime < 0)
      return;

    if (this->scriptTime >= this->scriptLength)
    {
      if (!this->loop)
      {
        return;
      }
      else
      {
        this->scriptTime = this->scriptTime - this->scriptLength;
        this->playStartTime = currentTime - this->scriptTime;
      }
    }

    for (unsigned int i = 0; i < this->trajInfo.size(); ++i)
    {
      if (this->trajInfo[i].startTime <= this->scriptTime &&
          this->trajInfo[i].endTime >= this->scriptTime)
      {
        tinfo = &this->trajInfo[i];
        break;
      }
    }

    if (tinfo == nullptr)
    {
      gzerr << "Trajectory not found at time [" << this->scriptTime << "]"
          << std::endl;
      return;
    }

    this->scriptTime = this->scriptTime - tinfo->startTime;
  }
  else
  {
    tinfo = this->customTrajectoryInfo.get();
  }

  /// at this point we are certain that a new frame will be animated
  this->prevFrameTime = currentTime;

  SkeletonAnimation *skelAnim = this->skelAnimation[tinfo->type];
  if (!skelAnim)
    return;

  auto skelMap = this->skelNodesMap[tinfo->type];

  ignition::math::Pose3d modelPose;
  std::map<std::string, ignition::math::Matrix4d> frame;
  if (!this->customTrajectoryInfo)
  {
    if (this->trajectories.find(tinfo->id) != this->trajectories.end())
    {
      common::PoseKeyFrame posFrame(0.0);
      this->trajectories[tinfo->id]->SetTime(this->scriptTime);
      this->trajectories[tinfo->id]->GetInterpolatedKeyFrame(posFrame);

      modelPose.Pos() = posFrame.Translation();
      modelPose.Rot() = posFrame.Rotation();
      if (this->lastTraj == tinfo->id)
        this->pathLength += fabs(this->lastPos.Distance(modelPose.Pos()));
      else
      {
        common::PoseKeyFrame *frame0 = dynamic_cast<common::PoseKeyFrame*>
          (this->trajectories[tinfo->id]->GetKeyFrame(0));
        ignition::math::Vector3d vector3Ign;
        vector3Ign = frame0->Translation();
        this->pathLength = fabs(modelPose.Pos().Distance(vector3Ign));
      }
      this->lastPos = modelPose.Pos();
    }
    if (this->interpolateX[tinfo->type] &&
          this->trajectories.find(tinfo->id) != this->trajectories.end())
    {
      frame = skelAnim->PoseAtX(this->pathLength,
                skelMap[this->skeleton->GetRootNode()->GetName()]);
    }
    else
    {
      frame = skelAnim->PoseAt(this->scriptTime);
    }
  }
  else
  {
    frame = skelAnim->PoseAt(this->scriptTime);
  }

  this->lastTraj = tinfo->id;

  ignition::math::Matrix4d rootTrans =
    frame[skelMap[this->skeleton->GetRootNode()->GetName()]];

  ignition::math::Vector3d rootPos = rootTrans.Translation();
  ignition::math::Quaterniond rootRot = rootTrans.Rotation();

  if (tinfo->translated)
    rootPos.X() = 0.0;
  ignition::math::Pose3d actorPose;
  actorPose.Pos() = modelPose.Pos() + modelPose.Rot().RotateVector(rootPos);
  if (!this->customTrajectoryInfo)
    actorPose.Rot() = modelPose.Rot() * rootRot;
  else
    actorPose.Rot() = modelPose.Rot() * this->GetWorldPose().Ign().Rot();

  ignition::math::Matrix4d rootM(actorPose.Rot());
  if (!this->customTrajectoryInfo)
    rootM.Translate(actorPose.Pos());

  frame[skelMap[this->skeleton->GetRootNode()->GetName()]] = rootM;

  this->SetPose(frame, skelMap, currentTime.Double());
}

//////////////////////////////////////////////////
void Actor::SetPose(std::map<std::string, ignition::math::Matrix4d> _frame,
      std::map<std::string, std::string> _skelMap, const double _time)
{
  msgs::PoseAnimation msg;
  msg.set_model_name(this->visualName);
  msg.set_model_id(this->visualId);

  ignition::math::Matrix4d modelTrans(ignition::math::Matrix4d::Identity);
  ignition::math::Pose3d mainLinkPose;

  if (this->customTrajectoryInfo)
    mainLinkPose.Rot() = this->worldPose.Ign().Rot();

  for (unsigned int i = 0; i < this->skeleton->GetNumNodes(); ++i)
  {
    SkeletonNode *bone = this->skeleton->GetNodeByHandle(i);
    SkeletonNode *parentBone = bone->GetParent();
    ignition::math::Matrix4d transform(ignition::math::Matrix4d::Identity);
    if (_frame.find(_skelMap[bone->GetName()]) != _frame.end())
      transform = _frame[_skelMap[bone->GetName()]];
    else
      transform = bone->Transform();

    LinkPtr currentLink = this->GetChildLink(bone->GetName());
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
      if (!this->customTrajectoryInfo)
        mainLinkPose = bonePose;
    }
    else
    {
      bone_pose->mutable_position()->CopyFrom(msgs::Convert(bonePose.Pos()));
      bone_pose->mutable_orientation()->CopyFrom(msgs::Convert(bonePose.Rot()));
      LinkPtr parentLink = this->GetChildLink(parentBone->GetName());
      math::Pose parentPose = parentLink->GetWorldPose();
      math::Matrix4 parentTrans(parentPose.rot.GetAsMatrix4());
      parentTrans.SetTranslate(parentPose.pos);
      transform = (parentTrans * transform).Ign();
    }

    msgs::Pose *link_pose = msg.add_pose();
    link_pose->set_name(currentLink->GetScopedName());
    link_pose->set_id(currentLink->GetId());
    ignition::math::Pose3d linkPose = transform.Pose() - mainLinkPose;
    link_pose->mutable_position()->CopyFrom(msgs::Convert(linkPose.Pos()));
    link_pose->mutable_orientation()->CopyFrom(msgs::Convert(linkPose.Rot()));
    currentLink->SetWorldPose(transform.Pose(), true, false);
  }

  msgs::Time *stamp = msg.add_time();
  stamp->CopyFrom(msgs::Convert(_time));

  msgs::Pose *model_pose = msg.add_pose();
  model_pose->set_name(this->GetScopedName());
  model_pose->set_id(this->GetId());
  if (!this->customTrajectoryInfo)
  {
    model_pose->mutable_position()->CopyFrom(msgs::Convert(mainLinkPose.Pos()));
    model_pose->mutable_orientation()->CopyFrom(
        msgs::Convert(mainLinkPose.Rot()));
  }
  else
  {
    model_pose->mutable_position()->CopyFrom(
        msgs::Convert(this->worldPose.Ign().Pos()));
    model_pose->mutable_orientation()->CopyFrom(
        msgs::Convert(this->worldPose.Ign().Rot()));
  }

  if (this->bonePosePub && this->bonePosePub->HasConnections())
    this->bonePosePub->Publish(msg);
  if (!this->customTrajectoryInfo)
    this->SetWorldPose(mainLinkPose, true, false);
}

//////////////////////////////////////////////////
void Actor::Fini()
{
  this->ResetCustomTrajectory();
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
void Actor::SetScriptTime(const double _time)
{
  this->scriptTime = _time;
}

//////////////////////////////////////////////////
double Actor::ScriptTime() const
{
  return this->scriptTime;
}

//////////////////////////////////////////////////
const Actor::SkeletonAnimation_M &Actor::SkeletonAnimations() const
{
  return this->skelAnimation;
}

//////////////////////////////////////////////////
void Actor::SetCustomTrajectory(TrajectoryInfoPtr &_trajInfo)
{
  this->customTrajectoryInfo = _trajInfo;
}

//////////////////////////////////////////////////
void Actor::ResetCustomTrajectory()
{
  this->customTrajectoryInfo.reset();
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
  if (this->skinFile.empty())
  {
    gzerr << "Can't add an actor visual without a skin file." << std::endl;
    return;
  }

  // Add visual
  sdf::ElementPtr visualSdf = _linkSdf->AddElement("visual");

  // Set name
  visualSdf->GetAttribute("name")->Set(_name);

  // Set pose
  sdf::ElementPtr visualPoseSdf = visualSdf->GetElement("pose");
  visualPoseSdf->Set(_pose);

  // Set mesh geometry (skin file)
  sdf::ElementPtr geomVisSdf = visualSdf->GetElement("geometry");
  sdf::ElementPtr meshSdf = geomVisSdf->GetElement("mesh");
  meshSdf->GetElement("uri")->Set(this->skinFile);
  meshSdf->GetElement("scale")->Set(ignition::math::Vector3d(this->skinScale,
      this->skinScale, this->skinScale));
}

/////////////////////////////////////////////////
void Actor::SetSelfCollide(bool /*_self_collide*/)
{
  // Actors don't support self collide
}

/////////////////////////////////////////////////
bool Actor::GetSelfCollide() const
{
  return false;
}

/////////////////////////////////////////////////
void Actor::SetWindMode(bool /*_enable*/)
{
  // Actors don't support wind mode
}

/////////////////////////////////////////////////
bool Actor::WindMode() const
{
  return false;
}

//////////////////////////////////////////////////
TrajectoryInfo::TrajectoryInfo()
  : id(0), type(""), duration(0.0), startTime(0.0), endTime(0.0),
  translated(false)
{
}
