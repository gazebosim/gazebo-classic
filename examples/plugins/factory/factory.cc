/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

namespace gazebo
{
class Factory : public WorldPlugin
{
  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    // Option 1: Insert model from file via function call.
    // The filename must be in the GAZEBO_MODEL_PATH environment variable.
    _parent->InsertModelFile("model://box");

    // Option 2: Insert model from string via function call.
    // Insert a sphere model from string
    sdf::SDF sphereSDF;
    sphereSDF.SetFromString(
       "<sdf version ='1.4'>\
          <model name ='sphere'>\
            <pose>1 0 0 0 0 0</pose>\
            <link name ='link'>\
              <pose>0 0 .5 0 0 0</pose>\
              <collision name ='collision'>\
                <geometry>\
                  <sphere><radius>0.5</radius></sphere>\
                </geometry>\
              </collision>\
              <visual name ='visual'>\
                <geometry>\
                  <sphere><radius>0.5</radius></sphere>\
                </geometry>\
              </visual>\
            </link>\
          </model>\
        </sdf>");
    // Demonstrate using a custom model name.
    sdf::ElementPtr model = sphereSDF.root->GetElement("model");
    model->GetAttribute("name")->SetFromString("unique_sphere");
    _parent->InsertModelSDF(sphereSDF);

    // Option 3: Insert model from file via message passing.
    {
      // Create a new transport node
      transport::NodePtr node(new transport::Node());

      // Initialize the node with the world name
      node->Init(_parent->GetName());

      // Create a publisher on the ~/factory topic
      transport::PublisherPtr factoryPub =
      node->Advertise<msgs::Factory>("~/factory");

      // Create the message
      msgs::Factory msg;

      // Model file to load
      msg.set_sdf_filename("model://cylinder");

      // Pose to initialize the model to
      msgs::Set(msg.mutable_pose(),
      math::Pose(math::Vector3(1, -2, 0), math::Quaternion(0, 0, 0)));

      // Send the message
      factoryPub->Publish(msg);
    }
  }
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(Factory)
}
