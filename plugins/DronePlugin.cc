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

#include <algorithm>
#include <string>


#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/OculusCamera.hh"

#include "gazebo/math/Helpers.hh"

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Assert.hh"
#include "gazebo/util/Joystick.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "plugins/DronePlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(DronePlugin)

/////////////////////////////////////////////////
DronePlugin::DronePlugin()
{
  this->joy = new util::Joystick();
}

/////////////////////////////////////////////////
void DronePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
  this->model = _model;
  this->joy->Init(0);

  /*physics::ShapePtr shape = this->model->GetWorld()->GetModel(
      "heightmap")->GetLink("link")->GetCollision("collision")->GetShape();
  this->heightmap = boost::dynamic_pointer_cast<physics::HeightmapShape>(shape);
  */

  this->testRay = boost::dynamic_pointer_cast<physics::RayShape>(
      this->model->GetWorld()->GetPhysicsEngine()->CreateShape(
        "ray", physics::CollisionPtr()));


  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&DronePlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void DronePlugin::OnUpdate()
{
  msgs::Joysticks msg;
  if (this->joy->Poll(msg))
  {
    if (msg.joy_size() > 0)
    {
      for (int i = 0; i < msg.joy(0).analog_axis_size(); ++i)
      {
        if (msg.joy(0).analog_axis(i).index() == 4)
        {
          // Forward motion
          this->velocity.x = msg.joy(0).analog_axis(i).value()/(-32768.0);
        }
        else if (msg.joy(0).analog_axis(i).index() == 1)
        {
          // Up/down
          this->velocity.z = msg.joy(0).analog_axis(i).value()/(-32768.0);
        }

        else if (msg.joy(0).analog_axis(i).index() == 0)
        {
          // Yaw
          this->yawSpeed = msg.joy(0).analog_axis(i).value()/(-32768.0);
        }

      }
      // std::cout << msg.DebugString();
    }

    //std::cout << "Velocity[" << this->velocity << "]\n";
  }

  math::Pose pose = this->model->GetWorldPose();

  pose.pos = pose.rot.RotateVector(
      this->velocity * math::Vector3(0.02,0.02,0.01)) + pose.pos;

  math::Vector3 rpy = pose.rot.GetAsEuler();
  rpy.z += this->yawSpeed * 0.001;
  pose.rot.SetFromEuler(rpy);

  this->testRay->SetPoints(pose.pos - math::Vector3(0,0,0.5),
                           pose.pos - math::Vector3(0,0,100));
  std::string entityName;
  double dist;
  this->testRay->GetIntersection(dist, entityName);

  double height = pose.pos.z - dist;
  // Limit the maximum height
  pose.pos.z = math::clamp(pose.pos.z, height+0.2, height+40.0);

  this->model->SetWorldPose(pose);
}
