/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#include <regex>

#include <gazebo/common/Assert.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>

#include <ignition/msgs.hh>
#include <ignition/transport.hh>

#include "JointControlPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(JointControlPlugin)

/////////////////////////////////////////////////
void JointControlPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "Model pointer is null");
  GZ_ASSERT(_sdf, "SDF pointer is null");

  std::vector<std::string> jointNames;
  for (auto joint : _model->GetJoints())
  {
    jointNames.push_back(joint->GetScopedName());
  }

  ignition::transport::Node node;
  std::string modelName = _model->GetScopedName();
  modelName = std::regex_replace(modelName, std::regex("::"), "/");
  ignition::transport::Node::Publisher jointPub =
      node.Advertise<ignition::msgs::JointCmd>("/" + modelName + "/joint_cmd");

  // Loop over controller definitions
  sdf::ElementPtr child = _sdf->GetFirstElement();
  while (child)
  {
    if (child->GetName() == "controller")
    {
      if (child->HasAttribute("type"))
      {
        ignition::msgs::JointCmd msg;
        std::string controllerType = child->Get<std::string>("type");

        if (controllerType == "force")
        {
          if (child->HasElement("target"))
          {
            msg.set_force(child->Get<double>("target"));
          }
          if (child->HasElement("pid_gains"))
          {
            gzwarn << "PID gains not used for force controllers.\n";
          }
        }
        else if (controllerType == "position")
        {
          if (child->HasElement("target"))
          {
            msg.mutable_position()->set_target(child->Get<double>("target"));
          }
          if (child->HasElement("pid_gains"))
          {
            auto gains = child->Get<ignition::math::Vector3d>("pid_gains");
            msg.mutable_position()->set_p_gain(gains.X());
            msg.mutable_position()->set_i_gain(gains.Y());
            msg.mutable_position()->set_d_gain(gains.Z());
          }
        }
        else if (controllerType == "velocity")
        {
          if (child->HasElement("target"))
          {
            msg.mutable_velocity()->set_target(child->Get<double>("target"));
          }
          if (child->HasElement("pid_gains"))
          {
            auto gains = child->Get<ignition::math::Vector3d>("pid_gains");
            msg.mutable_velocity()->set_p_gain(gains.X());
            msg.mutable_velocity()->set_i_gain(gains.Y());
            msg.mutable_velocity()->set_d_gain(gains.Z());
          }
        }
        else
        {
          gzerr << "Unrecognized controller type \""
                << controllerType << "\".\n";
          continue;
        }

        // Loop over joints
        sdf::ElementPtr grandchild = child->GetFirstElement();
        while (grandchild)
        {
          if (grandchild->GetName() == "joint")
          {
            // Find joint(s) matching given name or regex
            std::regex exp(_model->GetScopedName() + "::" +
                grandchild->Get<std::string>());
            std::vector<std::string> matches(jointNames.size());
            auto iter = std::copy_if(jointNames.begin(), jointNames.end(),
                matches.begin(), [&](const std::string &_name)
                {
                    return std::regex_match(_name, exp);
                });
            matches.resize(std::distance(matches.begin(), iter));

            if (matches.empty())
            {
              gzwarn << "No joints found matching \""
                     << grandchild->Get<std::string>() << "\".\n";
            }
            for (const auto match : matches)
            {
              msg.set_name(match);
              jointPub.Publish(msg);
            }
          }
          grandchild = grandchild->GetNextElement();
        }
      }
      else
      {
        gzerr << "Controller element missing required attribute \"type\".\n ";
      }
    }
    else
    {
      gzwarn << "Unexpected element \"" << child->GetName()
             << "\" in plugin block.\n";
    }
    child = child->GetNextElement();
  }
}
