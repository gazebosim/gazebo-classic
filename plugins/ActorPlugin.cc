/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#include "gazebo/math/Rand.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "plugins/ActorPlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(ActorPlugin)

/////////////////////////////////////////////////
ActorPlugin::ActorPlugin()
{
}

/////////////////////////////////////////////////
void ActorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world = this->actor->GetWorld();

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ActorPlugin::OnUpdate, this, _1)));

  this->velocity = 0.8;

  // Read in the first target location
  if (_sdf->HasElement("target"))
    this->target = _sdf->Get<math::Vector3>("target");
  else
    this->target = math::Vector3(0, -5, 1.2138);

  // Read in the target weight
  if (_sdf->HasElement("target_weight"))
    this->targetWeight = _sdf->Get<double>("target_weight");
  else
    this->targetWeight = 1.15;

  // Read in the obstacle weight
  if (_sdf->HasElement("obstacle_weight"))
    this->obstacleWeight = _sdf->Get<double>("obstacle_weight");
  else
    this->obstacleWeight = 1.5;

  // Read in the animation factor (applied in the OnUpdate function).
  if (_sdf->HasElement("animation_factor"))
    this->animationFactor = _sdf->Get<double>("animation_factor");
  else
    this->animationFactor = 4.5;

  // Add our own name to models we should ignore when avoiding obstacles.
  this->ignoreModels.push_back(this->actor->GetName());

  // Read in the other obstacles to ignore
  if (_sdf->HasElement("ignore_obstacles"))
  {
    sdf::ElementPtr modelElem =
      _sdf->GetElement("ignore_obstacles")->GetElement("model");
    while (modelElem)
    {
      this->ignoreModels.push_back(modelElem->Get<std::string>());
      modelElem = modelElem->GetNextElement("model");
    }
  }
}

/////////////////////////////////////////////////
void ActorPlugin::ChooseNewTarget()
{
  math::Vector3 newTarget(this->target);
  while ((newTarget - this->target).GetLength() < 2.0)
  {
    newTarget.x = math::Rand::GetDblUniform(-3, 3.5);
    newTarget.y = math::Rand::GetDblUniform(-10, 2);

    for (unsigned int i = 0; i < this->world->GetModelCount(); ++i)
    {
      double dist = (this->world->GetModel(i)->GetWorldPose().pos
          - newTarget).GetLength();
      if (dist < 2.0)
      {
        newTarget = this->target;
        break;
      }
    }

  }
  this->target = newTarget;
}

/////////////////////////////////////////////////
void ActorPlugin::HandleObstacles(math::Vector3 &_pos)
{
  for (unsigned int i = 0; i < this->world->GetModelCount(); ++i)
  {
    physics::ModelPtr model = this->world->GetModel(i);
    if (std::find(this->ignoreModels.begin(), this->ignoreModels.end(),
          model->GetName()) == this->ignoreModels.end())
    {
      math::Vector3 offset = model->GetWorldPose().pos -
        this->actor->GetWorldPose().pos;
      double modelDist = offset.GetLength();
      if (modelDist < 4.0)
      {
        double invModelDist = this->obstacleWeight / modelDist;
        offset.Normalize();
        offset *= invModelDist;
        _pos -= offset;
      }
    }
  }
}

/////////////////////////////////////////////////
void ActorPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  // Time delta
  double dt = (_info.simTime - this->lastUpdate).Double();

  math::Pose pose = this->actor->GetWorldPose();
  math::Vector3 pos = this->target - pose.pos;
  math::Vector3 rpy = pose.rot.GetAsEuler();

  double distance = pos.GetLength();

  // Choose a new target position if the actor has reached its current
  // target.
  if (distance < 0.3)
  {
    this->ChooseNewTarget();
    pos = this->target - pose.pos;
    distance = pos.GetLength();
  }

  // Normalize the direction vector, and apply the target weight
  pos = pos.Normalize() * this->targetWeight;

  // Adjust the direction vector by avoiding obstacles
  this->HandleObstacles(pos);

  // Compute the yaw orientation
  math::Angle yaw = atan2(pos.y, pos.x) + 1.5707 - rpy.z;
  yaw.Normalize();

  // Rotate in place, instead of jumping.
  if (std::abs(yaw.Radian()) > GZ_DTOR(10))
  {
    pose.rot = math::Quaternion(1.5707, 0, rpy.z+yaw.Radian()*0.001);
  }
  else
  {
    pose.pos += pos * this->velocity * dt;
    pose.rot = math::Quaternion(1.5707, 0, rpy.z+yaw.Radian());
  }

  // Make sure the actor stays within bounds
  pose.pos.x = std::max(-3.0, std::min(3.5, pose.pos.x));
  pose.pos.y = std::max(-10.0, std::min(2.0, pose.pos.y));
  pose.pos.z = 1.2138;

  // Distance traveled is used to coordinate motion with the walking
  // animation
  double distanceTraveled = (pose.pos -
      this->actor->GetWorldPose().pos).GetLength();

  this->actor->SetWorldPose(pose);
  this->actor->SetScriptTime(this->actor->GetScriptTime()+distanceTraveled*
      this->animationFactor);
  this->lastUpdate = _info.simTime;
}
