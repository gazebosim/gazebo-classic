/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#include <boost/thread/recursive_mutex.hpp>
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
#include "gazebo/physics/Actor.hh"
#include "gazebo/physics/PhysicsIface.hh"

#include "gazebo/transport/Node.hh"

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
  this->pathLength = 0.0;
  this->lastTraj = 1e+5;
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
  sdf::ElementPtr skinSdf = _sdf->GetElement("skin");
  this->skinFile = skinSdf->Get<std::string>("filename");
  this->skinScale = skinSdf->Get<double>("scale");

  MeshManager::Instance()->Load(this->skinFile);
  std::string actorName = _sdf->Get<std::string>("name");

/*  double radius = 1.0;
  unsigned int pointNum = 32;
  for (unsigned int i = 0; i < pointNum; i++)
  {
    double angle = (2 * i * M_PI) / pointNum;
    double x = radius * sin(angle);
    double y = radius * cos(angle);
    if (math::equal(x, 0.0))
      x = 0;
    if (math::equal(y, 0.0))
      y = 0;
    std::cerr << x << " " << y << " 0 0 0 " << angle << "\n";
  }   */

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

//    this->AddSphereInertia(linkSdf, math::Pose(), 1.0, 0.01);
//    this->AddSphereCollision(linkSdf, actorName + "_pose_col",
//                                             math::Pose(), 0.02);
    // this->AddBoxVisual(linkSdf, actorName + "_pose_vis", math::Pose(),
    //                   math::Vector3(0.05, 0.05, 0.05), "Gazebo/White",
    //                   Color::White);
    this->AddActorVisual(linkSdf, actorName + "_visual", math::Pose());
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
        pose = ignition::math::Pose3d();
      linkPose->Set(pose);

      /// FIXME hardcoded inertia of a sphere with mass 1.0 and radius 0.01
      this->AddSphereInertia(linkSdf, math::Pose(), 1.0, 0.01);

      /// FIXME hardcoded collision to sphere with radius 0.02
      this->AddSphereCollision(linkSdf, bone->GetName() + "_collision",
                       math::Pose(), 0.02);

      /// FIXME hardcoded visual to red sphere with radius 0.02
      if (bone->IsRootNode())
      {
        this->AddSphereVisual(linkSdf, bone->GetName() + "__SKELETON_VISUAL__",
                            math::Pose(), 0.02, "Gazebo/Blue", Color::Blue);
      }
      else
        if (bone->GetChildCount() == 0)
        {
            this->AddSphereVisual(linkSdf, bone->GetName() +
                            "__SKELETON_VISUAL__", math::Pose(), 0.02,
                            "Gazebo/Yellow", Color::Yellow);
        }
        else
          this->AddSphereVisual(linkSdf, bone->GetName() +
                            "__SKELETON_VISUAL__", math::Pose(), 0.02,
                            "Gazebo/Red", Color::Red);

      for (unsigned int i = 0; i < bone->GetChildCount(); ++i)
      {
        SkeletonNode *curChild = bone->GetChild(i);

        math::Vector3 dir = curChild->ModelTransform().Translation() -
            bone->ModelTransform().Translation();
        double length = dir.GetLength();

        if (!math::equal(length, 0.0))
        {
          ignition::math::Vector3d r =
            curChild->Transform().Translation();
          ignition::math::Vector3d linkPos =
            ignition::math::Vector3d(r.X() / 2.0, r.Y() / 2.0, r.Z() / 2.0);
          double theta = atan2(dir.y, dir.x);
          double phi = acos(dir.z / length);

          ignition::math::Pose3d bonePose(linkPos,
              ignition::math::Quaterniond(0.0, phi, theta));
          bonePose.Rot() = pose.Rot().Inverse() * bonePose.Rot();

          this->AddBoxVisual(linkSdf, bone->GetName() + "_" +
            curChild->GetName() + "__SKELETON_VISUAL__", bonePose,
            math::Vector3(0.02, 0.02, length), "Gazebo/Green", Color::Green);
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
         gzerr << "No actor visual message found.";
    }
    else
    {
      gzerr << "No actor link found.";
    }
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
        gzwarn << "Resource not found for trajectory of type " <<
                  trajSdf->Get<std::string>("type") << "\n";
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

      std::map<double, math::Pose> points;

      if (trajSdf->HasElement("waypoint"))
      {
        sdf::ElementPtr wayptSdf = trajSdf->GetElement("waypoint");
        while (wayptSdf)
        {
          points[wayptSdf->Get<double>("time")] =
                                          wayptSdf->Get<math::Pose>("pose");
          wayptSdf = wayptSdf->GetNextElement("waypoint");
        }

        std::map<double, math::Pose>::reverse_iterator last = points.rbegin();
        std::stringstream animName;
        animName << tinfo.type << "_" << tinfo.id;
        common::PoseAnimation *anim = new common::PoseAnimation(animName.str(),
                                                          last->first, false);
        this->trajInfo[idx].duration = last->first;
        this->trajInfo[idx].translated = true;

        for (std::map<double, math::Pose>::iterator pIter = points.begin();
              pIter != points.end(); ++pIter)
        {
          common::PoseKeyFrame *key;
          if (pIter == points.begin() && !math::equal(pIter->first, 0.0))
          {
            key = anim->CreateKeyFrame(0.0);
            key->Translation(pIter->second.pos.Ign());
            key->Rotation(pIter->second.rot.Ign());
          }
          key = anim->CreateKeyFrame(pIter->first);
          key->Translation(pIter->second.pos.Ign());
          key->Rotation(pIter->second.rot.Ign());
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
  double scriptTime = 0.0;
  if (!this->skelAnimation.empty())
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
      this->trajInfo[i].startTime = scriptTime;
      scriptTime += this->trajInfo[i].duration;
      this->trajInfo[i].endTime = scriptTime;
    }
  }
  this->scriptLength = scriptTime;
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
              " skeletons are not compatible.\n";
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
  /// TODO: Reducing to 20 Hz. Because there were memory corruption
  /// and segmentation faults. Possibly due to some dangling pointers
  /// in pose message processing. This will need a proper fix. Just a
  /// workaround for now.
  if ((currentTime - this->prevFrameTime).Double() < (1.0 / 20.0))
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

  for (unsigned int i = 0; i < this->trajInfo.size(); ++i)
    if (this->trajInfo[i].startTime <= scriptTime &&
          this->trajInfo[i].endTime >= scriptTime)
    {
      tinfo = this->trajInfo[i];
      break;
    }

  scriptTime = scriptTime - tinfo.startTime;

  SkeletonAnimation *skelAnim = this->skelAnimation[tinfo.type];
  std::map<std::string, std::string> skelMap = this->skelNodesMap[tinfo.type];

  ignition::math::Pose3d modelPose;
  std::map<std::string, ignition::math::Matrix4d> frame;
  if (this->trajectories.find(tinfo.id) != this->trajectories.end())
  {
    common::PoseKeyFrame posFrame(0.0);
    this->trajectories[tinfo.id]->SetTime(scriptTime);
    this->trajectories[tinfo.id]->GetInterpolatedKeyFrame(posFrame);

    modelPose.Pos() = posFrame.Translation();
    modelPose.Rot() = posFrame.Rotation();

    if (this->lastTraj == tinfo.id)
      this->pathLength += fabs(this->lastPos.Distance(modelPose.Pos()));
    else
    {
      common::PoseKeyFrame *frame0 = dynamic_cast<common::PoseKeyFrame*>
        (this->trajectories[tinfo.id]->GetKeyFrame(0));
      ignition::math::Vector3d vector3Ign;
      vector3Ign = frame0->Translation();
      this->pathLength = fabs(modelPose.Pos().Distance(vector3Ign));
    }
    this->lastPos = modelPose.Pos();
  }
  if (this->interpolateX[tinfo.type] &&
        this->trajectories.find(tinfo.id) != this->trajectories.end())
  {
    frame = skelAnim->PoseAtX(this->pathLength,
              skelMap[this->skeleton->GetRootNode()->GetName()]);
  }
  else
    frame = skelAnim->PoseAt(scriptTime);

  this->lastTraj = tinfo.id;

  ignition::math::Matrix4d rootTrans =
    frame[skelMap[this->skeleton->GetRootNode()->GetName()]];

  ignition::math::Vector3d rootPos = rootTrans.Translation();
  ignition::math::Quaterniond rootRot = rootTrans.Rotation();

  if (tinfo.translated)
    rootPos.X() = 0.0;
  ignition::math::Pose3d actorPose;
  actorPose.Pos() = modelPose.Pos() + modelPose.Rot().RotateVector(rootPos);
  actorPose.Rot() = modelPose.Rot() * rootRot;

  ignition::math::Matrix4d rootM(actorPose.Rot());
  rootM.Translate(actorPose.Pos());

  frame[skelMap[this->skeleton->GetRootNode()->GetName()]] = rootM;

  this->SetPose(frame, skelMap, currentTime.Double());

  this->lastScriptTime = scriptTime;
}

//////////////////////////////////////////////////
void Actor::SetPose(std::map<std::string, ignition::math::Matrix4d> _frame,
      std::map<std::string, std::string> _skelMap, double _time)
{
  msgs::PoseAnimation msg;
  msg.set_model_name(this->visualName);
  msg.set_model_id(this->visualId);

  ignition::math::Matrix4d modelTrans(ignition::math::Matrix4d::Identity);
  ignition::math::Pose3d mainLinkPose;

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
  model_pose->mutable_position()->CopyFrom(msgs::Convert(mainLinkPose.Pos()));
  model_pose->mutable_orientation()->CopyFrom(
      msgs::Convert(mainLinkPose.Rot()));

  if (this->bonePosePub && this->bonePosePub->HasConnections())
    this->bonePosePub->Publish(msg);
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
const sdf::ElementPtr Actor::GetSDF()
{
  return Model::GetSDF();
}

//////////////////////////////////////////////////
void Actor::AddSphereInertia(sdf::ElementPtr _linkSdf,
                             const math::Pose &_pose,
                             double _mass, double _radius)
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
void Actor::AddSphereCollision(sdf::ElementPtr _linkSdf,
                               const std::string &_name,
                               const math::Pose &_pose,
                               double _radius)
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
void Actor::AddSphereVisual(sdf::ElementPtr _linkSdf, const std::string &_name,
            const math::Pose &_pose, double _radius,
            const std::string &_material, const common::Color &_ambient)
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
void Actor::AddBoxVisual(sdf::ElementPtr _linkSdf, const std::string &_name,
    const math::Pose &_pose, const math::Vector3 &_size,
    const std::string &_material, const common::Color &_ambient)
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
void Actor::AddActorVisual(sdf::ElementPtr _linkSdf, const std::string &_name,
                           const math::Pose &_pose)
{
  sdf::ElementPtr visualSdf = _linkSdf->AddElement("visual");
  visualSdf->GetAttribute("name")->Set(_name);
  sdf::ElementPtr visualPoseSdf = visualSdf->GetElement("pose");
  visualPoseSdf->Set(_pose);
  sdf::ElementPtr geomVisSdf = visualSdf->GetElement("geometry");
  sdf::ElementPtr meshSdf = geomVisSdf->GetElement("mesh");
  meshSdf->GetElement("uri")->Set(this->skinFile);
  meshSdf->GetElement("scale")->Set(math::Vector3(this->skinScale,
      this->skinScale, this->skinScale));
}

//////////////////////////////////////////////////
TrajectoryInfo::TrajectoryInfo()
  : id(0), type(""), duration(0.0), startTime(0.0), endTime(0.0),
  translated(false)
{
}
