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
#include <gazebo/physics/Model.hh>
#include <memory>

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

class KeysToCmdVelPluginPrivate {
  public: KeysToCmdVelPluginPrivate() : keys(new CmdVelKeyboardControls)
  {
  }

  virtual ~KeysToCmdVelPluginPrivate()
  {
  }

  /// \brief Get the currently set linear speed.
  /// \return The linear speed.
  public: virtual double Linear() const = 0;

  /// \brief Get the currently set angular speed.
  /// \return The angular speed.
  public: virtual double Angular() const = 0;

  /// \brief Set desired linear speed.
  /// \param _linear The desired linear speed.
  public: virtual void SetLinear(double _linear) = 0;

  /// \brief Set desired angular speed.
  /// \param _angular The desired angular speed.
  public: virtual void SetAngular(double _angular) = 0;

  /// \brief Initialize the message publisher.
  public: virtual void InitPublisher() = 0;

  /// \brief Publish the cmd_vel message.
  public: virtual void Publish() = 0;

  /// \brief Stores information about each tracked key.
  public: std::unique_ptr<CmdVelKeyboardControls> keys;

  /// \brief The topic to which cmd_vel messages should be published.
  public: std::string cmdVelTopic;

  /// \brief Minimum linear velocity (for backwards driving, negative) (m/s).
  public: double minLinearVel = -1.0;

  /// \brief Maximum linear velocity (for forward driving, positive) (m/s).
  public: double maxLinearVel = 1.0;

  /// \brief Maximum angular velocity (positive value) (rad/s).
  public: double maxAngularVel = 1.0;

  /// \brief The value to add/subtract every time the linear velocity key is
  /// pressed (strictly positive value, m/s).
  public: double linearIncrement = 0.5;

  /// \brief The value to add/subtract every time the angular velocity key is
  /// pressed (strictly positive value, m/s).
  public: double angularIncrement = 0.5;

  /// \brief Node for communication.
  public: transport::NodePtr node;

  /// \brief Subscribe to keyboard messages.
  public: transport::SubscriberPtr keyboardSub;

  /// \brief Publish cmd_vel messages.
  public: transport::PublisherPtr cmdVelPub;
};

class KeysToCmdVelPluginPrivatePose : public KeysToCmdVelPluginPrivate
{
  public: KeysToCmdVelPluginPrivatePose() :
    keyboardControlMessage(new msgs::Pose)
  {
    msgs::Set(this->keyboardControlMessage->mutable_position(),
              ignition::math::Vector3d::Zero);
    msgs::Set(this->keyboardControlMessage->mutable_orientation(),
              ignition::math::Quaterniond::Identity);
  }

  double Linear() const override
  {
    return this->keyboardControlMessage->position().x();
  }
  double Angular() const override
  {
    return msgs::ConvertIgn(
        this->keyboardControlMessage->orientation()).Euler().Z();
  }

  void SetLinear(const double _linear) override
  {
    this->keyboardControlMessage->mutable_position()->set_x(_linear);
  }

  void SetAngular(const double _angular) override
  {
    const auto yaw = ignition::math::Quaterniond::EulerToQuaternion(
        0, 0, _angular);
    msgs::Set(this->keyboardControlMessage->mutable_orientation(), yaw);
  }

  void InitPublisher() override
  {
    this->cmdVelPub = this->node->Advertise<msgs::Pose>(this->cmdVelTopic);
  }

  void Publish() override
  {
    this->cmdVelPub->Publish(*this->keyboardControlMessage);
  }

  /// \brief The message to be sent that is updated by keypresses.
  public: msgs::PosePtr keyboardControlMessage;
};

class KeysToCmdVelPluginPrivateTwist : public KeysToCmdVelPluginPrivate
{
  public: KeysToCmdVelPluginPrivateTwist() :
    keyboardControlMessage(new msgs::Twist)
  {
    msgs::Set(this->keyboardControlMessage->mutable_linear(),
              ignition::math::Vector3d::Zero);
    msgs::Set(this->keyboardControlMessage->mutable_angular(),
              ignition::math::Vector3d::Zero);
  }

  double Linear() const override
  {
    return this->keyboardControlMessage->linear().x();
  }
  double Angular() const override
  {
    return this->keyboardControlMessage->angular().z();
  }

  void SetLinear(const double _linear) override
  {
    this->keyboardControlMessage->mutable_linear()->set_x(_linear);
  }

  void SetAngular(const double _angular) override
  {
    this->keyboardControlMessage->mutable_angular()->set_z(_angular);
  }

  void InitPublisher() override
  {
    this->cmdVelPub = this->node->Advertise<msgs::Twist>(this->cmdVelTopic);
  }

  void Publish() override
  {
    this->cmdVelPub->Publish(*this->keyboardControlMessage);
  }

  /// \brief The message to be sent that is updated by keypresses.
  public: msgs::TwistPtr keyboardControlMessage;
};
}

/////////////////////////////////////////////////
KeysToCmdVelPlugin::KeysToCmdVelPlugin()
{
}

/////////////////////////////////////////////////
KeysToCmdVelPlugin::~KeysToCmdVelPlugin()
{
}

/////////////////////////////////////////////////
void KeysToCmdVelPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  bool publishAsTwist;
  this->LoadParam(_sdf, "publish_as_twist", publishAsTwist, false);

  if (publishAsTwist)
  {
    this->dataPtr = std::unique_ptr<KeysToCmdVelPluginPrivateTwist>(
        new KeysToCmdVelPluginPrivateTwist);
  }
  else
  {
    this->dataPtr = std::unique_ptr<KeysToCmdVelPluginPrivatePose>(
        new KeysToCmdVelPluginPrivatePose);
  }

  this->LoadParam(_sdf, "cmd_vel_topic", this->dataPtr->cmdVelTopic,
                  "~/cmd_vel");
  this->LoadParam(_sdf, "max_linear_vel", this->dataPtr->maxLinearVel, 1.0);
  this->LoadParam(_sdf, "min_linear_vel", this->dataPtr->minLinearVel,
                  -this->dataPtr->maxLinearVel);
  this->LoadParam(_sdf, "linear_increment",
      this->dataPtr->linearIncrement, 0.5);
  this->LoadParam(_sdf, "max_angular_vel", this->dataPtr->maxAngularVel, 1.0);
  this->LoadParam(_sdf, "angular_increment",
      this->dataPtr->angularIncrement, 0.5);

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
        {"stop", this->dataPtr->keys->stop},
        {"accelerate", this->dataPtr->keys->accelerate},
        {"decelerate", this->dataPtr->keys->decelerate},
        {"left", this->dataPtr->keys->left},
        {"right", this->dataPtr->keys->right},
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
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();

  this->dataPtr->keyboardSub = this->dataPtr->node->Subscribe(
      "~/keyboard/keypress", &KeysToCmdVelPlugin::OnKeyPress, this, true);

  this->dataPtr->InitPublisher();
}

/////////////////////////////////////////////////
void KeysToCmdVelPlugin::Reset()
{
  this->dataPtr->SetLinear(0);
  this->dataPtr->SetAngular(0);
}

/////////////////////////////////////////////////
void KeysToCmdVelPlugin::OnKeyPress(ConstAnyPtr &_msg)
{
  const auto key = static_cast<unsigned int>(_msg->int_value());

  double linearVel = 0., angularVel = 0.;
  bool linearVelSet = false, angularVelSet = false;

  if (std::find(this->dataPtr->keys->stop.begin(),
                this->dataPtr->keys->stop.end(), key) !=
    this->dataPtr->keys->stop.end())
  {
    linearVel = 0.;
    linearVelSet = true;

    angularVel = 0.;
    angularVelSet = true;
  }
  else
  {
    if (std::find(this->dataPtr->keys->accelerate.begin(),
                  this->dataPtr->keys->accelerate.end(), key) !=
      this->dataPtr->keys->accelerate.end())
    {
      linearVel = this->dataPtr->maxLinearVel;
      linearVelSet = true;
    }
    else if (std::find(this->dataPtr->keys->decelerate.begin(),
                       this->dataPtr->keys->decelerate.end(), key) !=
      this->dataPtr->keys->decelerate.end())
    {
      linearVel = this->dataPtr->minLinearVel;
      linearVelSet = true;
    }

    if (linearVelSet)
    {
      const auto oldLinearVel = this->dataPtr->Linear();

      if (!ignition::math::equal(linearVel, oldLinearVel))
      {
        const auto increment = ignition::math::signum(linearVel) *
            this->dataPtr->linearIncrement;
        linearVel = ignition::math::clamp(oldLinearVel + increment,
          this->dataPtr->minLinearVel, this->dataPtr->maxLinearVel);
      }
    }

    if (std::find(this->dataPtr->keys->left.begin(),
                  this->dataPtr->keys->left.end(), key) !=
      this->dataPtr->keys->left.end())
    {
      angularVel = -this->dataPtr->maxAngularVel;
      angularVelSet = true;
    }
    else if (std::find(this->dataPtr->keys->right.begin(),
                       this->dataPtr->keys->right.end(), key)
      != this->dataPtr->keys->right.end())
    {
      angularVel = this->dataPtr->maxAngularVel;
      angularVelSet = true;
    }

    if (angularVelSet)
    {
      const auto oldAngularVel = this->dataPtr->Angular();

      if (!ignition::math::equal(angularVel, oldAngularVel))
      {
        const auto increment = ignition::math::signum(angularVel) *
            this->dataPtr->angularIncrement;
        angularVel = ignition::math::clamp(oldAngularVel + increment,
            -this->dataPtr->maxAngularVel, this->dataPtr->maxAngularVel);
      }
    }
  }

  if (linearVelSet)
  {
    this->dataPtr->SetLinear(linearVel);
  }

  if (angularVelSet)
  {
    this->dataPtr->SetAngular(angularVel);
  }

  if (linearVelSet || angularVelSet)
  {
    this->dataPtr->Publish();
  }
}
