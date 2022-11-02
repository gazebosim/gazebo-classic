/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#include <sstream>
#include <limits>
#include <algorithm>

#include <ignition/math/Color.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/math/Matrix4.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

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

/// \brief Private data for Actor class
class gazebo::physics::ActorPrivate
{
  /// \brief True if the animation is loaded from BVH file
  public: bool bvhFile = false;

  /// \brief Translations to align BVH skeleton to DAE skin
  public: std::map<std::string, ignition::math::Matrix4d>
      translationAligner;

  /// \brief Rotations to align BVH skeleton to DAE skin
  public: std::map<std::string, ignition::math::Matrix4d>
      rotationAligner;

  /// \brief Last animated frame
  public: std::map<std::string, ignition::math::Matrix4d> lastFrame;

  /// \brief Last map associating skeleton nodes from skin and animation
  public: std::map<std::string, std::string> lastSkelMap;
};

using namespace gazebo;
using namespace physics;
using namespace common;

//////////////////////////////////////////////////
Actor::Actor(BasePtr _parent)
  : Model(_parent), dataPtr(new ActorPrivate)
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
  // Name
  auto actorName = _sdf->Get<std::string>("name");
  this->SetName(actorName);

  // Parse skin
  if (_sdf->HasElement("skin"))
  {
    // Only load skeleton animations if we get a skeleton from the skin
    if (this->LoadSkin(_sdf->GetElement("skin")) && this->skeleton)
    {
      // If there are no user-defined animations, but skin has animation
      if (!_sdf->HasElement("animation") && !this->skinFile.empty() &&
          this->skeleton->GetAnimation(0))
      {
        auto animElem = _sdf->AddElement("animation");

        // Get name from trajectory
        if (_sdf->HasElement("script") &&
            _sdf->GetElement("script")->HasElement("trajectory"))
        {
          auto trajName = _sdf->GetElement("script")->GetElement("trajectory")->
              Get<std::string>("type");
          animElem->GetAttribute("name")->Set(trajName);
        }

        // Get file from skin
        animElem->GetElement("filename")->Set(this->skinFile);
      }

      if (_sdf->HasElement("animation"))
      {
        sdf::ElementPtr animSdf = _sdf->GetElement("animation");
        while (animSdf)
        {
          this->LoadAnimation(animSdf);
          animSdf = animSdf->GetNextElement("animation");
        }
      }
      else
      {
        gzwarn << "The skin provided doesn't contain animations, and no other "
            << "animations were specified. The actor's skeleton will not be "
            << "animated." << std::endl;
      }
    }
  }

  // Load script containing trajectory waypoints
  if (_sdf->HasElement("script"))
    this->LoadScript(_sdf->GetElement("script"));

  // Load all links, including the new ones added when loading the skin
  Model::Load(_sdf);

  // If there is a skin, check that the skin visual was created and save its id
  std::string actorLinkName = actorName + "::" + actorName + "_pose";
  LinkPtr actorLinkPtr = Model::GetLink(actorLinkName);
  if (actorLinkPtr)
  {
    msgs::Visual actorVisualMsg = actorLinkPtr->GetVisualMessage(
        this->visualName);
    if (actorVisualMsg.has_id())
      this->visualId = actorVisualMsg.id();
    else
    {
      gzerr << "Message for actor visual [" << actorLinkName << "] not found."
          << std::endl;
    }
  }

  // Advertise skeleton pose info
  this->bonePosePub = this->node->Advertise<msgs::PoseAnimation>(
                                       "~/skeleton_pose/info", 10);
}

//////////////////////////////////////////////////
bool Actor::LoadSkin(sdf::ElementPtr _skinSdf)
{
  this->skinFile = _skinSdf->Get<std::string>("filename");
  this->skinScale = _skinSdf->Get<double>("scale");

  MeshManager::Instance()->Load(this->skinFile);
  if (!MeshManager::Instance()->HasMesh(this->skinFile))
  {
    gzwarn << "Couldn't find mesh [" << this->skinFile << "]. " <<
        "Not loading skin." << std::endl;
    return false;
  }

  this->mesh = MeshManager::Instance()->GetMesh(this->skinFile);
  if (!this->mesh || !this->mesh->HasSkeleton())
  {
    gzwarn << "Collada file [" << this->skinFile <<
        "] does not contain skeletal animation." << std::endl;
    return false;
  }

  this->skeleton = this->mesh->GetSkeleton();
  this->SetScale({this->skinScale, this->skinScale, this->skinScale});

  auto actorName = this->GetName();

  // Create a link to hold the skin visual
  auto linkSdf = _skinSdf->GetParent()->GetElement("link");
  linkSdf->GetAttribute("name")->Set(actorName + "_pose");
  linkSdf->GetElement("gravity")->Set(false);
  linkSdf->GetElement("self_collide")->Set(false);

  std::string actorLinkName = actorName + "::" + actorName + "_pose";
  this->visualName = actorLinkName + "::" + actorName + "_visual";
  this->AddActorVisual(linkSdf, actorName + "_visual",
      ignition::math::Pose3d::Zero);

  // Create spherical links for each skeleton node
  auto nodes = this->skeleton->GetNodes();
  for (auto iter : nodes)
  {
    SkeletonNode *bone = iter.second;

    // Add link element
    linkSdf = _skinSdf->GetParent()->AddElement("link");

    // Set default properties
    linkSdf->GetAttribute("name")->Set(bone->GetName());
    linkSdf->GetElement("gravity")->Set(false);
    linkSdf->GetElement("self_collide")->Set(false);

    // Set pose
    ignition::math::Pose3d pose(bone->ModelTransform().Translation(),
                                bone->ModelTransform().Rotation());
    if (bone->IsRootNode())
      pose = ignition::math::Pose3d::Zero;

    linkSdf->GetElement("pose")->Set(pose);

    // FIXME hardcoded inertia of a sphere with mass 1.0 and radius 0.01
    // Do we even need inertial info in an actor?
    this->AddSphereInertia(linkSdf, ignition::math::Pose3d::Zero, 1.0, 0.01);

    // FIXME hardcoded visual to red sphere with radius 0.02
    if (bone->IsRootNode())
    {
      this->AddSphereVisual(linkSdf, bone->GetName() + "__SKELETON_VISUAL__",
          ignition::math::Pose3d::Zero, 0.02, "Gazebo/Blue",
          ignition::math::Color::Blue);
    }
    else if (bone->GetChildCount() == 0)
    {
      this->AddSphereVisual(linkSdf, bone->GetName() +
          "__SKELETON_VISUAL__", ignition::math::Pose3d::Zero, 0.02,
          "Gazebo/Yellow", ignition::math::Color::Yellow);
    }
    else
    {
      this->AddSphereVisual(linkSdf, bone->GetName() +
          "__SKELETON_VISUAL__", ignition::math::Pose3d::Zero, 0.02,
          "Gazebo/Red", ignition::math::Color::Red);
    }

    // Create a box visual representing each bone
    for (unsigned int i = 0; i < bone->GetChildCount(); ++i)
    {
      SkeletonNode *curChild = bone->GetChild(i);

      ignition::math::Vector3d dir =
          curChild->ModelTransform().Translation() -
          bone->ModelTransform().Translation();
      double length = dir.Length();

      if (!ignition::math::equal(length, 0.0))
      {
        ignition::math::Vector3d r = curChild->Transform().Translation();
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
          "Gazebo/Green", ignition::math::Color::Green);
        this->AddBoxCollision(linkSdf,
            bone->GetName() + "_" + curChild->GetName() + "_collision",
            bonePose, ignition::math::Vector3d(0.02, 0.02, length));
      }
    }
  }
  return true;
}

//////////////////////////////////////////////////
void Actor::LoadScript(sdf::ElementPtr _sdf)
{
  this->loop = _sdf->Get<bool>("loop");
  this->startDelay = _sdf->Get<double>("delay_start");
  this->autoStart = _sdf->Get<bool>("auto_start");
  this->active = this->autoStart;

  // Load all trajectories
  if (_sdf->HasElement("trajectory"))
  {
    sdf::ElementPtr trajSdf = _sdf->GetElement("trajectory");
    while (trajSdf)
    {
      auto trajType = trajSdf->Get<std::string>("type");
      double tension = trajSdf->Get<double>("tension", 0.0).first;

      TrajectoryInfo tinfo;
      tinfo.id = trajSdf->Get<int>("id");
      tinfo.type = trajType;

      // Place trajectory into vector according to id order
      auto iter = this->trajInfo.begin();
      while (iter != this->trajInfo.end())
      {
        if (iter->id > tinfo.id)
          break;
        ++iter;
      }

      unsigned int idx = iter - this->trajInfo.begin();
      this->trajInfo.insert(iter, tinfo);

      // Waypoints
      if (trajSdf->HasElement("waypoint"))
      {
        // Fill a map with waypoints time and pose
        std::map<double, ignition::math::Pose3d> points;
        sdf::ElementPtr wayptSdf = trajSdf->GetElement("waypoint");
        while (wayptSdf)
        {
          points[wayptSdf->Get<double>("time")] =
              wayptSdf->Get<ignition::math::Pose3d>("pose");
          wayptSdf = wayptSdf->GetNextElement("waypoint");
        }

        // Get total trajectory duration (last waypoint's time)
        auto last = points.rbegin();

        std::stringstream animName;
        animName << tinfo.type << "_" << tinfo.id;
        common::PoseAnimation *anim = new common::PoseAnimation(animName.str(),
            last->first, false, tension);

        // Create a keyframe for each point
        for (auto pIter = points.begin(); pIter != points.end(); ++pIter)
        {
          common::PoseKeyFrame *key;
          // Force first point always to start at 0s
          if (pIter == points.begin() &&
              !ignition::math::equal(pIter->first, 0.0))
          {
            key = anim->CreateKeyFrame(0.0);
          }
          else
          {
            key = anim->CreateKeyFrame(pIter->first);
          }

          key->Translation(pIter->second.Pos());
          key->Rotation(pIter->second.Rot());
        }

        // `trajInfo` holds information like start, end and duration
        this->trajInfo[idx].duration = last->first;
        this->trajInfo[idx].translated = true;

        // `trajectories` holds information about the animation itself
        this->trajectories[this->trajInfo[idx].id] = anim;
      }
      // No waypoints defined but there is skeleton animation of this type
      else if (this->skelAnimation.find(this->trajInfo[idx].type) !=
          this->skelAnimation.end())
      {
        this->trajInfo[idx].duration =
                this->skelAnimation[this->trajInfo[idx].type]->GetLength();
        this->trajInfo[idx].translated = false;
      }

      trajSdf = trajSdf->GetNextElement("trajectory");
    }
  }

  // If there are no trajectories, but there are animations, add a trajectory
  if (!this->skelAnimation.empty() && this->skelAnimation.begin()->second &&
      this->trajInfo.empty())
  {
    TrajectoryInfo tinfo;
    tinfo.id = 0;
    tinfo.type = this->skelAnimation.begin()->first;
    tinfo.startTime = 0.0;
    tinfo.duration = this->skelAnimation.begin()->second->GetLength();
    tinfo.endTime = tinfo.duration;
    tinfo.translated = false;
    this->trajInfo.push_back(tinfo);
    // Nothing goes into this->trajectories
  }

  // Finally, (re)fill the times for all trajectories so that they are in a
  // sequence
  double time = 0.0;
  for (unsigned int i = 0; i < this->trajInfo.size(); ++i)
  {
    this->trajInfo[i].startTime = time;
    time += this->trajInfo[i].duration;
    this->trajInfo[i].endTime = time;
  }
  this->scriptLength = time;
}

//////////////////////////////////////////////////
void Actor::LoadAnimation(sdf::ElementPtr _sdf)
{
  if (!this->skeleton)
  {
    gzwarn << "Animations need a skeleton defined first. " <<
        "Not loading animations" << std::endl;
    return;
  }

  std::string animName = _sdf->Get<std::string>("name");

  // Get filename and extension
  std::string animFile = _sdf->Get<std::string>("filename");

  std::string extension = animFile.substr(animFile.rfind(".") + 1,
      animFile.size());
  std::transform(extension.begin(), extension.end(), extension.begin(),
      ::tolower);

  // Get scale
  double animScale = _sdf->Get<double>("scale");

  // Load animation skeleton according to file type
  Skeleton *skel = nullptr;
  if (extension == "bvh")
  {
    BVHLoader loader;
    skel = loader.Load(animFile, animScale);
    this->dataPtr->bvhFile = true;
  }
  else if (extension == "dae")
  {
    MeshManager::Instance()->Load(animFile);

    const class Mesh *animMesh = nullptr;
    if (MeshManager::Instance()->HasMesh(animFile))
    {
      animMesh = MeshManager::Instance()->GetMesh(animFile);

      if (animMesh && animMesh->HasSkeleton())
      {
        skel = animMesh->GetSkeleton();
        skel->Scale(animScale);
      }
      else
      {
        gzwarn << "Mesh [" << animFile << "] is missing a skeleton."
            << std::endl;
      }
    }
    else
    {
      gzwarn << "Couldn't load animation file [" << animFile << "]"
          << std::endl;
    }
  }
  else
  {
    gzerr << "Unknown animation file extension [" << extension << "]"
        << std::endl;
  }

  if (!skel || skel->GetNumAnimations() == 0)
  {
    gzerr << "Failed to load animation [" << animName << "]" << std::endl;
    return;
  }

  bool compatible = true;

  // The skeleton coming from the skin and the skeleton coming from the
  // animation are compatible if they have the same number of nodes
  // and each node has the same number of children as their counterpart
  std::map<std::string, std::string> skelMap;
  if (this->skeleton->GetNumNodes() != skel->GetNumNodes())
    compatible = false;
  else
  {
    for (unsigned int i = 0; i < this->skeleton->GetNumNodes(); ++i)
    {
      SkeletonNode *skinNode = this->skeleton->GetNodeByHandle(i);
      SkeletonNode *animNode = skel->GetNodeByHandle(i);
      if (animNode->GetChildCount() != skinNode->GetChildCount())
      {
        compatible = false;
        break;
      }
      // If compatible, associate the animation node to the skin node
      else
        skelMap[skinNode->GetName()] = animNode->GetName();
    }
  }

  if (!compatible)
  {
    gzerr << "Skin and animation [" << animName <<
          "] skeletons are not compatible. " <<
          "Do they have the same number of nodes?" << std::endl;
    return;
  }

  if (this->dataPtr->bvhFile)
  {
    this->AlignBvh(skel, skelMap);
  }

  this->skelAnimation[animName] = skel->GetAnimation(0);
  this->interpolateX[animName] = _sdf->Get<bool>("interpolate_x");
  this->skelNodesMap[animName] = skelMap;
}

//////////////////////////////////////////////////
// Equivalent to ignition::common::Skeleton::AddBvhAnimation
void Actor::AlignBvh(Skeleton *_skel,
    const std::map<std::string, std::string> &_skelMap)
{
  // calculate translationAligner: aligner of initial bvh pose to initial dae
  // pose
  for (unsigned int i = 0; i < _skel->GetNumNodes(); ++i)
  {
    SkeletonNode *animNode = _skel->GetNodeByHandle(i);
    SkeletonNode *skinNode = this->skeleton->GetNodeByHandle(i);

    if (animNode->GetParent() != nullptr)
    {
      if (animNode->GetParent()->GetChildCount() > 1)
      {
        // parent link has multiple child links
        if (skinNode->Transform().Translation()
            == ignition::math::Vector3d::Zero)
        {
          // parent link is a virtual link (has no length)
          this->dataPtr->translationAligner[animNode->GetName()] =
              ignition::math::Matrix4d::Identity;
        }
      }
    }

    if (animNode->GetChildCount() == 0)
    {
      // if this is an end bone, then link i matrix is already calculated
      continue;
    }

    auto it = _skelMap.find(this->skeleton->GetRootNode()->GetName());
    if (it != _skelMap.end() && animNode->GetName() == it->second)
    {
      // if this is root, then some setup is needed to match bvh and dae
      this->dataPtr->translationAligner[animNode->GetName()] =
          ignition::math::Matrix4d(
          this->skeleton->GetRootNode()->Transform().Rotation());
      auto tmp = this->dataPtr->translationAligner[animNode->GetName()];
      tmp.SetTranslation(animNode->Transform().Translation());
      animNode->SetTransform(tmp, true);
    }

    if (animNode->GetChildCount() > 1)
    {
      // if link i has multiple children
      continue;
    }

    // else, which means link i only has a single child link i+1

    // get link i+1 posture direction in world coordinates
    auto relativeBvh = animNode->GetChild(0)->ModelTransform().Translation()
        - animNode->ModelTransform().Translation();

    auto relativeSkin = skinNode->GetChild(0)->ModelTransform().Translation()
          - skinNode->ModelTransform().Translation();

    if (relativeBvh == ignition::math::Vector3d::Zero ||
        relativeSkin == ignition::math::Vector3d::Zero)
    {
      // unexpected
      gzerr << "Duplicated joint found! This might cause some errors!\n";
      continue;
    }

    // calculate world coordinate rotation quaternion
    // (difference between link i+1 posture)
    auto n = relativeBvh.Cross(relativeSkin);
    double theta = asin(n.Length() / (relativeSkin.Length() *
        relativeBvh.Length()));

    // calculate bvh to dae of link i+1
    this->dataPtr->translationAligner[animNode->GetChild(0)->GetName()] =
        ignition::math::Matrix4d(skinNode->ModelTransform().Rotation())
        .Inverse()
        * ignition::math::Matrix4d(ignition::math::Quaterniond(n.Normalize(),
          theta))
        * ignition::math::Matrix4d(animNode->ModelTransform().Rotation());

    // fix bvh posture of all links until link i,
    // so that bvh and dae world posture matches
    auto tmp = ignition::math::Matrix4d(
        animNode->ModelTransform().Rotation()).Inverse();
    tmp = tmp * ignition::math::Matrix4d(
        ignition::math::Quaterniond(n.Normalize(), theta));
    tmp = tmp * ignition::math::Matrix4d(animNode->ModelTransform().Rotation());
    tmp.SetTranslation(animNode->Transform().Translation());
    animNode->SetTransform(tmp, true);
  }

  // calculate the rotationAligner : aligner of initial bvh pose to initial dae
  // pose
  for (unsigned int i = 0; i < _skel->GetNumNodes(); ++i)
  {
    SkeletonNode *animNode = _skel->GetNodeByHandle(i);
    SkeletonNode *skinNode = this->skeleton->GetNodeByHandle(i);

    auto it = _skelMap.find(this->skeleton->GetRootNode()->GetName());
    if (it != _skelMap.end() && animNode->GetName() == it->second)
    {
      // rotation should not be aligned with root
      this->dataPtr->rotationAligner[animNode->GetName()] =
          ignition::math::Matrix4d::Identity;
      continue;
    }

    // in case an aligner was not correctly calculated, set a value to prevent
    // nan
    if (this->dataPtr->translationAligner[animNode->GetName()] ==
        ignition::math::Matrix4d::Zero)
    {
      this->dataPtr->translationAligner[animNode->GetName()] =
          ignition::math::Matrix4d::Identity;
    }

    this->dataPtr->rotationAligner[animNode->GetName()] =
        ignition::math::Matrix4d(animNode->Transform().Rotation()).Inverse()
        * this->dataPtr->translationAligner[animNode->GetName()].Inverse()
        * ignition::math::Matrix4d(skinNode->Transform().Rotation());
  }
}

//////////////////////////////////////////////////
void Actor::Init()
{
  this->scriptTime = 0;
  this->prevFrameTime = this->world->SimTime();
  if (this->autoStart)
    this->Play();
  this->mainLink = this->GetChildLink(this->GetName() + "_pose");
}

//////////////////////////////////////////////////
void Actor::Play()
{
  this->active = true;
  this->playStartTime = this->world->SimTime();
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
  common::Time currentTime = this->world->SimTime();
  if (!this->active)
  {
    this->SetPose(this->dataPtr->lastFrame, this->dataPtr->lastSkelMap,
                  currentTime.Double());
    return;
  }

  if (this->skelAnimation.empty() && this->trajectories.empty())
    return;

  // do not refresh animation faster than 30 Hz sim time
  if ((currentTime - this->prevFrameTime).Double() < (1.0 / 30.0))
    return;

  // Get trajectory
  TrajectoryInfo *tinfo = nullptr;
  if (!this->customTrajectoryInfo)
  {
    this->scriptTime = currentTime.Double() - this->startDelay -
              this->playStartTime.Double();

    // waiting for delayed start
    if (this->scriptTime < 0)
    {
      this->SetPose(this->dataPtr->lastFrame, this->dataPtr->lastSkelMap,
                    currentTime.Double());
      return;
    }

    if (this->scriptTime >= this->scriptLength)
    {
      if (!this->loop)
      {
        this->active = false;
        return;
      }
      else
      {
        this->scriptTime = this->scriptTime - this->scriptLength;
        this->playStartTime = currentTime - this->scriptTime;
      }
    }

    // Pick trajectory which should be played at this time
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

  // at this point we are certain that a new frame will be animated
  this->prevFrameTime = currentTime;

  // Update global trajectory (not skeleton animation)
  ignition::math::Pose3d modelPose;
  if (!this->customTrajectoryInfo &&
      this->trajectories.find(tinfo->id) != this->trajectories.end())
  {
    // Get the pose keyframe calculated for this script time
    common::PoseKeyFrame posFrame(0.0);
    this->trajectories[tinfo->id]->SetTime(this->scriptTime);
    this->trajectories[tinfo->id]->GetInterpolatedKeyFrame(posFrame);

    modelPose.Pos() = posFrame.Translation();
    modelPose.Rot() = posFrame.Rotation();

    // Calculate the path length.
    // If we're still in the same trajectory, compare to last position
    if (this->lastTraj == tinfo->id)
    {
      this->pathLength += this->lastPos.Distance(modelPose.Pos());
    }
    // Otherwise, compare to first frame of this trajectory - ?
    else
    {
      auto frame0 = dynamic_cast<common::PoseKeyFrame *>
        (this->trajectories[tinfo->id]->GetKeyFrame(0));
      ignition::math::Vector3d vector3Ign = frame0->Translation();
      this->pathLength = modelPose.Pos().Distance(vector3Ign);
    }
    this->lastPos = modelPose.Pos();
  }

  SkeletonAnimation *skelAnim = this->skelAnimation[tinfo->type];

  // If there's no skeleton animation, we just update the global pose
  if (!skelAnim)
  {
    this->SetWorldPose(modelPose);
    return;
  }

  auto skelMap = this->skelNodesMap[tinfo->type];

  std::map<std::string, ignition::math::Matrix4d> frame;
  if (!this->customTrajectoryInfo)
  {
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

  ignition::math::Matrix4d rootTrans = ignition::math::Matrix4d::Identity;
  auto iter = frame.find(skelMap[this->skeleton->GetRootNode()->GetName()]);
  if (iter != frame.end())
  {
    rootTrans = frame[skelMap[this->skeleton->GetRootNode()->GetName()]];
  }

  ignition::math::Vector3d rootPos = rootTrans.Translation();
  ignition::math::Quaterniond rootRot = rootTrans.Rotation();
  // Zero root pos for BVH
  if (this->dataPtr->bvhFile)
  {
    rootPos = ignition::math::Vector3d::Zero;
  }

  if (tinfo->translated)
    rootPos.X() = 0.0;
  ignition::math::Pose3d actorPose;

  if (!this->customTrajectoryInfo)
  {
    actorPose.Pos() = modelPose.Pos() + modelPose.Rot().RotateVector(rootPos);
    actorPose.Rot() = modelPose.Rot() * rootRot;
  }
  else
  {
    actorPose.Pos() = this->WorldPose().Pos();
    actorPose.Rot() = this->WorldPose().Rot();
  }

  ignition::math::Matrix4d rootM(actorPose.Rot());

  rootM.SetTranslation(actorPose.Pos());

  // TODO: Possible bug here? Rotation changed after scaling. Maybe the
  // rotation algorithm is not suppose to work on non unit quaternion.
//    gzdbg << "before: " << rootM.Rotation() << std::endl;
//    rootM.Scale(this->skinScale, this->skinScale, this->skinScale);
//    auto scaleTrans = ignition::math::Matrix4d::Identity;
//    scaleTrans.Scale(this->skinScale, this->skinScale, this->skinScale);
//    rootM = scaleTrans * rootM;
//    gzdbg << "after: " << rootM.Rotation() << std::endl;

  // workaround for rotation bug
  rootM.SetTranslation(rootM.Translation() * this->skinScale);

  frame[skelMap[this->skeleton->GetRootNode()->GetName()]] = rootM;

  this->dataPtr->lastFrame = frame;
  this->dataPtr->lastSkelMap = skelMap;

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
  {
    mainLinkPose.Pos() = this->worldPose.Pos();
    mainLinkPose.Rot() = this->worldPose.Rot();
  }

  for (unsigned int i = 0; i < this->skeleton->GetNumNodes(); ++i)
  {
    SkeletonNode *bone = this->skeleton->GetNodeByHandle(i);
    SkeletonNode *parentBone = bone->GetParent();
    ignition::math::Matrix4d transform(ignition::math::Matrix4d::Identity);

    if (_frame.find(_skelMap[bone->GetName()]) != _frame.end())
    {
      if (this->dataPtr->bvhFile)
      {
        std::string tempStr(_skelMap[bone->GetName()]);
        transform = _frame[tempStr];

        if (bone->GetName() != this->skeleton->GetRootNode()->GetName())
        {
          ignition::math::Vector3d bvhOffset = transform.Translation();
          ignition::math::Vector3d daeOffset = bone->Transform().Translation();
          // scale bvh offset to dae link length
          transform.SetTranslation(daeOffset.Length() * bvhOffset.Normalize());
        }

        transform = this->dataPtr->translationAligner[tempStr] * transform *
            this->dataPtr->rotationAligner[tempStr];
      }
      else
      {
        transform = _frame[_skelMap[bone->GetName()]];
      }
    }
    else
    {
      transform = bone->Transform();
    }

    LinkPtr currentLink = this->GetChildLink(bone->GetName());
    ignition::math::Pose3d bonePose = transform.Pose();
    if (!bonePose.IsFinite())
    {
      gzerr << "ACTOR: " << _time << " " << bone->GetName()
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
      auto parentPose = parentLink->WorldPose();
      ignition::math::Matrix4d parentTrans(parentPose);
      transform = parentTrans * transform;
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
        msgs::Convert(this->worldPose.Pos()));
    model_pose->mutable_orientation()->CopyFrom(
        msgs::Convert(this->worldPose.Rot()));
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
void Actor::Reset()
{
  this->Stop();
  this->ResetCustomTrajectory();
  this->playStartTime = this->world->SimTime();
  this->pathLength = 0.0;
  this->lastTraj = 1e+5;
  this->Init();

  Model::Reset();
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
const common::Mesh *Actor::Mesh() const
{
  return this->mesh;
}

//////////////////////////////////////////////////
TrajectoryInfoPtr Actor::CustomTrajectory() const
{
  return this->customTrajectoryInfo;
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
    const ignition::math::Color &_ambient)
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
    const ignition::math::Color &_ambient)
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
void Actor::AddBoxCollision(const sdf::ElementPtr &_linkSdf,
    const std::string &_name, const ignition::math::Pose3d &_pose,
    const ignition::math::Vector3d &_size)
{
  sdf::ElementPtr collisionSdf = _linkSdf->AddElement("collision");
  collisionSdf->GetAttribute("name")->Set(_name);
  sdf::ElementPtr collisionPoseSdf = collisionSdf->GetElement("pose");
  collisionPoseSdf->Set(_pose);
  sdf::ElementPtr geomCollSdf = collisionSdf->GetElement("geometry");
  sdf::ElementPtr boxSdf = geomCollSdf->GetElement("box");
  boxSdf->GetElement("size")->Set(_size);
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
void Actor::SetWindMode(const bool /*_enable*/)
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

