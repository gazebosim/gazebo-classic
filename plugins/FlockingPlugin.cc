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

#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/World.hh"
#include "plugins/FlockingPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(FlockingPlugin)

/////////////////////////////////////////////////
FlockingPlugin::FlockingPlugin()
{
}

/////////////////////////////////////////////////
void FlockingPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
  this->model = _model;
  this->world = _model->GetWorld();

  // Collide with nothing
  for (auto &link : this->model->GetLinks())
    link->SetCollideMode("none");

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&FlockingPlugin::OnUpdate, this));

  this->linearSpeed = 2.5;
  this->angularSpeed = 8.5;
}

/////////////////////////////////////////////////
void FlockingPlugin::OnUpdate()
{
  math::Vector3 alignVel;
  math::Vector3 cohesionVel;
  math::Vector3 separationVel;

  unsigned int modelCount = 0;

  math::Pose myPose = this->model->GetWorldPose();

  for (auto const &iter : this->world->GetModels())
  {
    if (iter != this->model)
    {
      math::Pose pose = iter->GetWorldPose();
      double d = pose.pos.Distance(myPose.pos);

      if (d < 4)
      {
        modelCount++;
        alignVel += iter->GetLinearVel();
        cohesionVel += pose.pos;
        separationVel += (myPose.pos - pose.pos).Normalize() / d;
      }
    }
  }

  // Compute alignment component
  alignVel /= modelCount;
  alignVel.Normalize();
  alignVel *= 0.05;

  // Compute cohesion component
  cohesionVel /= modelCount;
  cohesionVel = (cohesionVel - myPose.pos).Normalize();
  cohesionVel *= 0.301;

  // Compute separation component
  separationVel /= modelCount;
  separationVel.Normalize();
  separationVel *= 0.3;

  // Get speed and direction in local frame.
  // speed = x component
  // direction = y component
  // z component is ignored
  math::Vector3 vel = myPose.rot.RotateVectorReverse(
      alignVel + separationVel + cohesionVel);

  // Get linear velocity in world frame
  math::Vector3 linearVel =
    myPose.rot.RotateVector(math::Vector3(vel.x, 0, 0)) * this->linearSpeed;

  // Get angular velocity in world frame
  math::Vector3 angularVel = math::Vector3(0, 0, vel.y) * this->angularSpeed;

  // Set the world linear velocity
  this->model->SetLinearVel(linearVel);

  // Set the angular velocity. Forcing only yaw to keep everything on the
  // ground plane.
  this->model->SetAngularVel(angularVel);
}
