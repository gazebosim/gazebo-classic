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
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sstream>

#include "model_move.hh"

using namespace gazebo;

ModelMove::ModelMove()
{ }

void ModelMove::move(const math::Vector3 &start, const math::Vector3 &end,
                     math::Vector3 &translation)
{
  int duration = floor(start.Distance(end.x, end.y, end.z));
  math::Vector3 diff = end - start;
  float x_step = diff.x / duration;
  float y_step = diff.y / duration;
  float z_step = diff.z / duration;
  int curr_frame = anim->GetKeyFrameCount();

  for (int i = 1; i <= duration; i++)
  {
    gazebo::common::PoseKeyFrame * key = anim->CreateKeyFrame(i+curr_frame);
    key->SetTranslation(math::Vector3(
         translation.x + x_step * i,
         translation.y + y_step * i,
         translation.z + z_step * i));
    key->SetRotation(math::Quaternion(0, 0, 0));
  }

  translation.Set(translation.x + x_step*duration,
                  translation.y + y_step*duration,
                  translation.z + z_step*duration);
}

void ModelMove::initiateMove()
{
  // get distance from starting point to the first of the goals
  float path_length = start_position.Distance(this->path_goals[0].pos);

  // to calculate the full distance, add the distance between goals
  for (int i = 0; i < this->path_goals.size()-1; i++)
    path_length += path_goals[i].pos.Distance(path_goals[i+1].pos);

  // create the animation
  this->anim =
    gazebo::common::PoseAnimationPtr(
        new gazebo::common::PoseAnimation("test", path_length+1, false));

  gazebo::common::PoseKeyFrame *key;

  // set starting location of the box
  key = anim->CreateKeyFrame(0);
  key->SetTranslation(math::Vector3(0, 0, 0));
  key->SetRotation(math::Quaternion(0, 0, 0));

  math::Vector3 translation = math::Vector3(0, 0, 0);

  // Move to the start_position to first goal
  move(start_position, path_goals[0].pos, translation);

  for (int i = 0; i < this->path_goals.size()-1; i++)
    move(path_goals[i].pos, path_goals[i+1].pos, translation);

  // set the animation
  this->model->SetAnimation(anim);
}

void ModelMove::getPathMsg(ConstPoseAnimationPtr &msg)
{
  gzmsg << "[model_move] Received path message" << std::endl;

  // Store message poses into the path_goals and launch movement
  for (int i = 0; i < msg->pose_size(); i++)
    this->path_goals.push_back(gazebo::msgs::Convert(msg->pose(i)));

  initiateMove();
}

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
    this->path_goals.push_back(poseElem->Get<math::Pose>());
    poseElem = poseElem->GetNextElement("pose");
  }

  GZ_ASSERT(this->path_goals.size() > 0, "path_goals should not be zero");
  return true;
}

void ModelMove::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Store the pointer to the model
  this->model = _parent;
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());

  // Get parameters from sdf, if provided
  if (_sdf->HasElement("goals"))
  {
    if (this->LoadGoalsFromSDF(_sdf->GetElement("goals")))
    {
      // Ready to start the move. Store the initial pose of the model
      // and call initiateMove
      sdf::Vector3 sdf_pose =
        _sdf->GetParent()->GetElement("pose")->Get<sdf::Pose>().pos;
      this->start_position =
        math::Vector3(sdf_pose.x, sdf_pose.y, sdf_pose.z);

      initiateMove();
    }
    else
    {
      gzerr << "[model_move] Problems on loading goals from sdf "
            << "made the movement impossible" << std::endl;
    }
  }

  // Create the subscriber
  std::string path_topic_name = std::string("~/") + _parent->GetName()
                                + "/model_move";
  pathSubscriber = node->Subscribe(path_topic_name, &ModelMove::getPathMsg,
                                   this);
  gzmsg << "[model_move] Subscribed to receive paths in: "<< path_topic_name
        << std::endl;
}
