/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include "gazebo/physics/physics.hh"
#include "gazebo/gazebo.hh"

#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/msgs/MessageTypes.hh"
#include "gazebo/common/Time.hh"

namespace gazebo
{
  class RayTest : public SensorPlugin
  {
    public: void Load(sensors::SensorPtr &/*_parent*/, sdf::ElementPtr &_sdf)
    {
      // Get then name of the parent model
      std::string modelName = _sdf->GetParent()->Get<std::string>("name");

      // Get the world name.
      std::string worldName = _sdf->GetWorldName();
      physics::WorldPtr world = physics::get_world(worldName);

      // Get a pointer to the model
      this->model = world->GetModelByName(modelName);

      // Error message if the model couldn't be found
      if (!this->model)
        gzerr << "Unable to get parent model\n";

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&RayTest::OnUpdate, this));
      gzdbg << "plugin model name: " << modelName << "\n";


      this->node = transport::NodePtr(new transport::Node());
      this->node->Init(worldName);
      this->statsSub =
        this->node->Subscribe("~/world_stats", &RayTest::OnStats, this);
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // do something on update
      // gzdbg << "plugin update\n";
    }

    public: void OnStats(
                const boost::shared_ptr<msgs::WorldStatistics const> &_msg)
    {
      this->simTime  = msgs::Convert(_msg->sim_time());

      math::Pose pose;
      pose.pos.x = 0.5*sin(0.01*this->simTime.Double());
      math::Pose orig_pose = this->model->GetWorldPose();

      if (this->simTime.Double() > 20.0)
        this->model->SetWorldPose(pose);

      gzdbg << "plugin simTime [" << this->simTime.Double()
            << "] update pose [" << pose.pos.x << "] orig pose ["
            << orig_pose << "]\n";
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // subscribe to world stats
    private: transport::NodePtr node;
    private: transport::SubscriberPtr statsSub;
    private: common::Time simTime;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(RayTest)
}

