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
  class PR2PoseTest : public ModelPlugin
  {
    public: void Load(physics::ModelPtr &/*_parent*/, sdf::ElementPtr &_sdf)
    {
      // Get then name of the parent model
      std::string modelName = _sdf->GetParent()->Get<std::string>("name");

      // Get the world name.
      std::string worldName = _sdf->GetWorldName();
      this->world = physics::get_world(worldName);

      // Get a pointer to the model
      this->model = this->world->GetModelByName(modelName);

      // Error message if the model couldn't be found
      if (!this->model)
        gzerr << "Unable to get parent model\n";

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&PR2PoseTest::OnUpdate, this));
      gzdbg << "plugin model name: " << modelName << "\n";


      this->node = transport::NodePtr(new transport::Node());
      this->node->Init(worldName);
      this->statsSub = this->node->Subscribe("~/world_stats",
          &PR2PoseTest::OnStats, this);
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      this->simTime  = this->world->GetSimTime();

      math::Pose orig_pose = this->model->GetWorldPose();
      math::Pose pose = orig_pose;
      pose.pos.x = 5.0*sin(0.1*this->simTime.Double());
      pose.rot.SetFromEuler(math::Vector3(0, 0, 2.0*this->simTime.Double()));

      if (this->simTime.Double() > 10.0 && this->simTime.Double() < 300.0)
      {
        this->model->SetWorldPose(pose);
        printf("test plugin OnUpdate simTime [%f]", this->simTime.Double());
        printf(" update pose [%f, %f, %f:%f, %f, %f, %f] orig pose.x [%f]\n",
            pose.pos.x, pose.pos.y, pose.pos.z, pose.rot.x, pose.rot.y,
            pose.rot.z, pose.rot.w, orig_pose.pos.x);

        if (this->simTime.Double() > 20.0)
          this->model->SetWorldTwist(math::Vector3(0, 0, 0),
                                     math::Vector3(0, 0, 0), true);
      }
    }

    public: void OnStats(
                const boost::shared_ptr<msgs::WorldStatistics const> &/*_msg*/)
    {
      static double fake_time = 0;
      fake_time = fake_time + 0.2;

      math::Pose orig_pose = this->model->GetWorldPose();
      math::Pose pose = orig_pose;
      pose.pos.x = 0.5*sin(0.1*fake_time);
      pose.rot.SetFromEuler(math::Vector3(0, 0, fake_time));

      if (this->world->GetSimTime().Double() < 10.0)
      {
        this->model->SetWorldPose(pose);
        printf("test plugin OnStats simTime [%f] ", fake_time);
        printf("update pose [%f, %f, %f:%f, %f, %f, %f] orig pose.x [%f]\n",
            pose.pos.x, pose.pos.y, pose.pos.z, pose.rot.x, pose.rot.y,
            pose.rot.z, pose.rot.w, orig_pose.pos.x);
      }
    }

    // Pointer to the model
    private: physics::ModelPtr model;
    private: physics::WorldPtr world;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // subscribe to world stats
    private: transport::NodePtr node;
    private: transport::SubscriberPtr statsSub;
    private: common::Time simTime;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(PR2PoseTest)
}
