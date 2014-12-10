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
void ActorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world = this->actor->GetWorld();

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->actor->GetWorld()->GetName());

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ActorPlugin::OnUpdate, this, _1)));

  this->velocity = 0.8;
}

/////////////////////////////////////////////////
void ActorPlugin::Init()
{
  this->target = math::Vector3(0, -5, 1.24);
}

/////////////////////////////////////////////////
void ActorPlugin::ChooseNewTarget()
{
  math::Vector3 newTarget(this->target);
  while ((newTarget - this->target).GetLength() < 2.0)
  {
    newTarget.x = math::Rand::GetDblUniform(-3, 3.5);
    newTarget.y = math::Rand::GetDblUniform(-10, 2);
  }
  this->target = newTarget;
}

/////////////////////////////////////////////////
void ActorPlugin::HandleObstacles(math::Vector3 &_pos)
{
  double obstacleWeight = 1.0;

  for (unsigned int i = 0; i < this->world->GetModelCount(); ++i)
  {
    physics::ModelPtr model = this->world->GetModel(i);
    if (model->GetName().find("table") != std::string::npos)
    {
      math::Vector3 offset = model->GetWorldPose().pos -
        this->actor->GetWorldPose().pos;
      double modelDist = offset.GetLength();
      double invModelDist = obstacleWeight / modelDist;
      offset.Normalize();
      offset *= invModelDist;
      _pos -= offset;
    }
  }
}

/////////////////////////////////////////////////
void ActorPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  double targetWeight = 1.2;

  double dt = (_info.simTime - this->lastUpdate).Double();

  math::Pose pose = this->actor->GetWorldPose();
  math::Vector3 pos = this->target - pose.pos;
  double distance = pos.GetLength();

  // Choose a new target position if the actor has reached its current
  // target.
  if (distance < 0.1)
  {
    this->ChooseNewTarget();
    pos = this->target - pose.pos;
    pos.z = 0;
    distance = pos.GetLength();
  }

  pos.Normalize() * targetWeight;

  this->HandleObstacles(pos);

  /*while (pos.GetLength() < 0.4)
  {
    this->ChooseNewTarget();
    pos = this->target - pose.pos;
    distance = pos.GetLength();

    pos.Normalize() * targetWeight;
    this->HandleObstacles(pos);
  }*/
  std::cout << "Target[" << this->target << "] P[" << pose.pos << "] ";
  std::cout << "Pos[" << pos << "]\n";

  //pose.pos += pos.Normalize() * velocity * dt;
  pose.pos += pos * velocity * dt;
  pose.pos.z = 1.24;

  double yaw = atan2(pos.y, pos.x);
  pose.rot = math::Quaternion(1.5707, 0, yaw+1.5707);

  this->actor->SetWorldPose(pose);
  this->lastUpdate = _info.simTime;
}
