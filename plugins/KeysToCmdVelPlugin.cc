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
#include <gazebo/physics/Model.hh>

#include "KeysToCmdVelPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(KeysToCmdVelPlugin)

namespace gazebo
{
class CmdVelKeyboardControls
{
  /// \brief Key for zeroing-out the velocity (default is Space, Enter).
  public: std::vector<unsigned int> stop;

  /// \brief Accelerate key (default is up arrow).
  public: std::vector<unsigned int> accelerate;

  /// \brief Decelerate key (default is down arrow).
  public: std::vector<unsigned int> decelerate;

  /// \brief Left key (default is left arrow).
  public: std::vector<unsigned int> left;

  /// \brief Right key (default is right arrow).
  public: std::vector<unsigned int> right;

  public: CmdVelKeyboardControls()
  {
    this->stop.push_back(13);
    this->stop.push_back(32);

    this->accelerate.push_back(38);
    this->accelerate.push_back(16777235);

    this->decelerate.push_back(40);
    this->decelerate.push_back(16777237);

    this->left.push_back(37);
    this->left.push_back(16777234);

    this->right.push_back(39);
    this->right.push_back(16777236);
  }

  public: virtual ~CmdVelKeyboardControls() = default;
};
}

/////////////////////////////////////////////////
KeysToCmdVelPlugin::KeysToCmdVelPlugin()
  : keys(new CmdVelKeyboardControls), keyboardControlMessage(new msgs::Pose)
{
  msgs::Set(this->keyboardControlMessage->mutable_position(),
            ignition::math::Vector3d::Zero);
  msgs::Set(this->keyboardControlMessage->mutable_orientation(),
            ignition::math::Quaterniond::Identity);
}

/////////////////////////////////////////////////
KeysToCmdVelPlugin::~KeysToCmdVelPlugin()
{
}

/////////////////////////////////////////////////
void KeysToCmdVelPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->LoadParam(_sdf, "cmd_vel_topic", this->cmdVelTopic, "~/cmd_vel");
  this->LoadParam(_sdf, "max_linear_vel", this->maxLinearVel, 1.0);
  this->LoadParam(_sdf, "min_linear_vel", this->minLinearVel,
                  -this->maxLinearVel);
  this->LoadParam(_sdf, "max_angular_vel", this->maxAngularVel, 1.0);

  const auto keyControlsEmpty = !_sdf->HasElement("key_controls") ||
    _sdf->GetElement("key_controls")->GetFirstElement() == sdf::ElementPtr();

  gzmsg << this->handleName << " Plugin keyboard control enabled with "
        << (keyControlsEmpty ? "default" : "custom") << " key assignments for "
        << "model " << _model->GetName() << std::endl;

  // if the <key_controls> tag is empty (but present) keyboard control should
  // be enabled with the default key assignments
  if (!keyControlsEmpty)
  {
    // Mapping from XML keys to TrackedVehicleKeyboardControls members.
    const std::map<const std::string, std::vector<unsigned int> &>
      controlsMapping =
      {
        {"stop", this->keys->stop},
        {"accelerate", this->keys->accelerate},
        {"decelerate", this->keys->decelerate},
        {"left", this->keys->left},
        {"right", this->keys->right},
      };

    const auto keyControlsElem = _sdf->GetElement("key_controls");
    for (auto controlsPair : controlsMapping)
    {
      const std::string &controlKeyName = controlsPair.first;
      std::vector<unsigned int> &controlKeyList = controlsPair.second;

      controlKeyList.clear();
      if (keyControlsElem->HasElement(controlKeyName))
      {
        auto controlElem = keyControlsElem->GetElement(controlKeyName);
        while (controlElem != nullptr)
        {
          controlKeyList.push_back(controlElem->Get<unsigned int>());
          controlElem = controlElem->GetNextElement(controlKeyName);
        }
      }
      else
      {
        gzwarn << "Key " << controlKeyName << " has no assigned keycode." <<
               std::endl;
      }
    }
  }
}

/////////////////////////////////////////////////
void KeysToCmdVelPlugin::Init()
{
  // Initialize transport
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  this->keyboardSub = this->node->Subscribe("~/keyboard/keypress",
    &KeysToCmdVelPlugin::OnKeyPress, this, true);

  this->cmdVelPub = this->node->Advertise<msgs::Pose>(this->cmdVelTopic);
}

/////////////////////////////////////////////////
void KeysToCmdVelPlugin::Reset()
{
  this->keyboardControlMessage->mutable_position()->set_x(0.0);
  msgs::Set(this->keyboardControlMessage->mutable_orientation(),
            ignition::math::Quaterniond::Identity);
}

/////////////////////////////////////////////////
void KeysToCmdVelPlugin::OnKeyPress(ConstAnyPtr &_msg)
{
  const auto key = static_cast<const unsigned int>(_msg->int_value());

  double linearVel = 0., angularVel = 0.;
  bool linearVelSet = false, angularVelSet = false;

  auto &message = this->keyboardControlMessage;

  if (std::find(this->keys->stop.begin(), this->keys->stop.end(), key) !=
    this->keys->stop.end())
  {
    linearVel = 0.;
    linearVelSet = true;

    angularVel = 0.;
    angularVelSet = true;
  }
  else
  {
    if (std::find(
      this->keys->accelerate.begin(), this->keys->accelerate.end(), key) !=
      this->keys->accelerate.end())
    {
      linearVel = this->maxLinearVel;
      linearVelSet = true;
    }
    else if (std::find(
      this->keys->decelerate.begin(), this->keys->decelerate.end(), key) !=
      this->keys->decelerate.end())
    {
      linearVel = this->minLinearVel;
      linearVelSet = true;
    }

    if (linearVelSet)
    {
      const auto oldLinearVel = message->position().x();

      // For some reason, each keypress is sent twice to the topic, so we add
      // only a half of what should be added.
      if (!ignition::math::equal(linearVel, oldLinearVel))
      {
        const auto increment =
          (linearVel > 0) ? this->maxLinearVel : this->minLinearVel;
        linearVel = ignition::math::clamp(
          oldLinearVel + 0.5 * increment,
          this->minLinearVel, this->maxLinearVel);
      }
    }

    if (std::find(this->keys->left.begin(), this->keys->left.end(), key) !=
      this->keys->left.end())
    {
      angularVel = -this->maxAngularVel;
      angularVelSet = true;
    }
    else if (std::find(this->keys->right.begin(), this->keys->right.end(), key)
      != this->keys->right.end())
    {
      angularVel = this->maxAngularVel;
      angularVelSet = true;
    }

    if (angularVelSet)
    {
      const auto oldAngularVel =
        msgs::ConvertIgn(message->orientation()).Euler().Z();

      // For some reason, each keypress is sent twice to the topic, so we add
      // only a half of what should be added.
      if (!ignition::math::equal(angularVel, oldAngularVel))
      {
        angularVel = oldAngularVel +
          ignition::math::signum(angularVel) * 0.5 * this->maxAngularVel;
      }
    }
  }

  if (linearVelSet)
  {
    message->mutable_position()->set_x(linearVel);
  }

  if (angularVelSet)
  {
    auto yaw = ignition::math::Quaterniond::EulerToQuaternion(
      0, 0, angularVel);
    msgs::Set(message->mutable_orientation(), yaw);
  }

  if (linearVelSet || angularVelSet)
  {
    this->cmdVelPub->Publish(*message);
  }
}
