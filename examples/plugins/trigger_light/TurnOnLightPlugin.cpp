/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

/// \brief This file contains a gazebo plugin for the ContainPlugin tutorial.
/// Doxygen comments and PIMPL are omitted to reduce the amount of text.

#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>

#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/transport/Node.hh"

namespace gazebo
{
  class TurnOnLightPlugin : public WorldPlugin
  {
    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override
    {
      gzmsg << "Loading Example plugin\n";
      // Transport initialization
      this->gzNode = transport::NodePtr(new transport::Node());
      this->gzNode->Init();

      // Subscribe to ContainPlugin output
      std::string topic("contain_example/contain");
      std::function<void(const ignition::msgs::Boolean &)> cb =
          [=](const ignition::msgs::Boolean &_msg){
        TurnOnLightPlugin::OnContainPluginMsg(_msg);
      };
      const bool containSub = this->node.Subscribe(topic, cb);
      if (!containSub)
      {
        gzerr << "Failed to subscribe to [" << topic << "]\n";
      }

      // Make a publisher for the light topic
      this->lightPub = this->gzNode->Advertise<msgs::Light>("~/light/modify");
    }

    public: void OnContainPluginMsg(const ignition::msgs::Boolean &_msg)
    {
      msgs::Light lightMsg;
      lightMsg.set_name("post_light");
      // Turn light on when the entity enters the box, and off when it leaves
      if (_msg.data())
      {
        gzmsg << "Turning on light\n";
        lightMsg.set_range(15.0);
      }
      else
      {
        gzmsg << "Turning off light\n";
        lightMsg.set_range(0.0);
      }
      this->lightPub->Publish(lightMsg);
    }

    private: ignition::transport::Node node;
    private: transport::NodePtr gzNode;
    private: transport::PublisherPtr lightPub;
  };
  GZ_REGISTER_WORLD_PLUGIN(TurnOnLightPlugin);
}  // namespace gazebo

