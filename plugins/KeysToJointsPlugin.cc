/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#include <gazebo/common/PID.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/JointController.hh>
#include <gazebo/physics/Model.hh>
#include "KeysToJointsPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(KeysToJointsPlugin)

/////////////////////////////////////////////////
KeysToJointsPlugin::KeysToJointsPlugin()
{
}

/////////////////////////////////////////////////
KeysToJointsPlugin::~KeysToJointsPlugin()
{
}

/////////////////////////////////////////////////
void KeysToJointsPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->model = _model;
  auto controller = this->model->GetJointController();

  // Load params from SDF
  if (_sdf->HasElement("map"))
  {
    auto mapElem = _sdf->GetElement("map");
    while (mapElem)
    {
      auto jointName = mapElem->Get<std::string>("joint");
      auto joint = this->model->GetJoint(jointName);
      if (!joint)
      {
        gzwarn << "Can't find joint [" << jointName << "]" << std::endl;
      }
      else
      {
        KeyInfo info;
        info.key = mapElem->Get<int>("key");
        info.joint = joint;
        info.scale = mapElem->Get<double>("scale");
        info.type = mapElem->Get<std::string>("type");

        double kp = mapElem->Get<double>("kp");
        double ki = mapElem->Get<double>("ki");
        double kd = mapElem->Get<double>("kd");

        common::PID pid(kp, ki, kd);
        if (info.type == "position")
          controller->SetPositionPID(info.joint->GetScopedName(), pid);
        else if (info.type == "velocity")
          controller->SetVelocityPID(info.joint->GetScopedName(), pid);

        this->keys.push_back(info);
      }

      mapElem = mapElem->GetNextElement("map");
    }
  }

  // Initialize transport
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  this->keyboardSub =
      this->node->Subscribe("/gazebo/default/keyboard/keypress",
      &KeysToJointsPlugin::OnKeyPress, this, true);
}

/////////////////////////////////////////////////
void KeysToJointsPlugin::OnKeyPress(ConstAnyPtr &_msg)
{
  for (auto &key : this->keys)
  {
    if (_msg->int_value() != key.key)
      continue;

    auto controller = this->model->GetJointController();

    if (key.type == "position")
    {
      auto currPos = key.joint->GetAngle(0).Radian();
      controller->SetPositionTarget(key.joint->GetScopedName(),
          currPos + key.scale);
    }
    else if (key.type == "velocity")
    {
      controller->SetVelocityTarget(key.joint->GetScopedName(),
          key.scale);
    }
    else if (key.type == "force")
    {
      key.joint->SetForce(0, key.scale);
    }
  }
}

