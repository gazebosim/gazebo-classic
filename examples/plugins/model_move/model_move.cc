/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sstream>

#include "model_move.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelMove);

/////////////////////////////////////////////////
ModelMove::ModelMove()
{
}

/////////////////////////////////////////////////
void ModelMove::Move(const ignition::math::Vector3d &_start,
  const ignition::math::Vector3d &_end, ignition::math::Vector3d &_translation)
{
  int duration = floor(_start.Distance(_end));
  ignition::math::Vector3d diff = _end - _start;
  float xStep = diff.X() / duration;
  float yStep = diff.Y() / duration;
  float zStep = diff.Z() / duration;
  int currFrame = this->anim->GetKeyFrameCount();

  for (int i = 1; i <= duration; ++i)
  {
    gazebo::common::PoseKeyFrame *key = this->anim->CreateKeyFrame(
        i + currFrame);

    key->Translation(ignition::math::Vector3d(
         _translation.X() + xStep * i,
         _translation.Y() + yStep * i,
         _translation.Z() + zStep * i));
    key->Rotation(ignition::math::Quaterniond(0, 0, 0));
  }

  _translation.Set(_translation.X() + xStep * duration,
                   _translation.Y() + yStep * duration,
                   _translation.Z() + zStep * duration);
}

/////////////////////////////////////////////////
void ModelMove::InitiateMove()
{
  // get distance from starting point to the first of the goals
  float pathLength = this->startPosition.Distance(this->pathGoals[0].Pos());

  // to calculate the full distance, add the distance between goals
  for (unsigned int i = 0; i < this->pathGoals.size()-1; ++i)
    pathLength += this->pathGoals[i].Pos().Distance(this->pathGoals[i+1].Pos());

  // create the animation
  this->anim = gazebo::common::PoseAnimationPtr(
        new gazebo::common::PoseAnimation("test", pathLength + 1, false));

  gazebo::common::PoseKeyFrame *key;

  // set starting location of the box
  key = this->anim->CreateKeyFrame(0);
  key->Translation(ignition::math::Vector3d(0, 0, 0));
  key->Rotation(ignition::math::Quaterniond(0, 0, 0));

  ignition::math::Vector3d translation = ignition::math::Vector3d(0, 0, 0);

  // Move to the startPosition to first goal
  this->Move(this->startPosition, this->pathGoals[0].Pos(), translation);

  for (unsigned int i = 0; i < this->pathGoals.size()-1; ++i)
  {
    this->Move(
      this->pathGoals[i].Pos(), this->pathGoals[i+1].Pos(), translation);
  }

  // set the animation
  this->model->SetAnimation(this->anim);
}

/////////////////////////////////////////////////
void ModelMove::OnPathMsg(ConstPoseAnimationPtr &_msg)
{
  gzmsg << "[model_move] Received path message" << std::endl;

  // Store message poses into the pathGoals and launch movement
  for (unsigned int i = 0; i < _msg->pose_size(); ++i)
    this->pathGoals.push_back(gazebo::msgs::ConvertIgn(_msg->pose(i)));

  this->InitiateMove();
}

/////////////////////////////////////////////////
bool ModelMove::LoadGoalsFromSDF(const sdf::ElementPtr _sdf)
{
  gzmsg << "[model_move] Processing path goals defined in the SDF file"
        << std::endl;
  GZ_ASSERT(_sdf, "_sdf element is null");

  if (!_sdf->HasElement("pose"))
  {
    gzerr << "[model_move] SDF with goals tag but without pose/s element/s"
          << std::endl;
    return false;
  }

  sdf::ElementPtr poseElem = _sdf->GetElement("pose");

  while (poseElem)
  {
    this->pathGoals.push_back(poseElem->Get<ignition::math::Pose3d>());
    poseElem = poseElem->GetNextElement("pose");
  }

  GZ_ASSERT(this->pathGoals.size() > 0, "pathGoals should not be zero");

  return true;
}

/////////////////////////////////////////////////
void ModelMove::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Store the pointer to the model
  this->model = _parent;
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->Name());

  // Get parameters from sdf, if provided
  if (_sdf->HasElement("goals"))
  {
    if (this->LoadGoalsFromSDF(_sdf->GetElement("goals")))
    {
      // Ready to start the move. Store the initial pose of the model
      // and call initiateMove
      ignition::math::Vector3d sdfPose =
        _sdf->GetParent()->GetElement("pose")
            ->Get<ignition::math::Pose3d>().Pos();
      this->startPosition =
        ignition::math::Vector3d(sdfPose.X(), sdfPose.Y(), sdfPose.Z());

      this->InitiateMove();
    }
    else
    {
      gzerr << "[model_move] Problems on loading goals from sdf "
            << "made the movement impossible" << std::endl;
    }
  }

  // Create the subscriber
  std::string pathTopicName = std::string("~/") + _parent->GetName()
    + "/model_move";
  this->pathSubscriber = node->Subscribe(
      pathTopicName, &ModelMove::OnPathMsg, this);
  gzmsg << "[model_move] Subscribed to receive paths in: "<< pathTopicName
        << std::endl;
}
