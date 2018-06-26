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

#include <memory>
#include <string>
#include <vector>

#include <ignition/math/Color.hh>

#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "plugins/LedPlugin.hh"

namespace gazebo
{
  class LedSettingPrivate
  {
    /// \brief The transparency when the ligth is off.
    public: double transparency;

    /// \brief The emissive color.
    public: ignition::math::Color emissiveColor;

    /// \brief The pointer to publisher to send a command to update a visual.
    public: transport::PublisherPtr pubVisual;

    /// \brief A message holding a Visual message.
    public: msgs::Visual msg;

    /// \brief True if <visual> element exists.
    public: bool visualExists;
  };

  class LedPluginPrivate
  {
    /// \brief The pointer to node for communication.
    public: transport::NodePtr node;

    /// \brief The pointer to publisher to send a command to the visual.
    public: transport::PublisherPtr pubVisual;
  };
}

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(LedPlugin)

//////////////////////////////////////////////////
LedSetting::LedSetting(
  const sdf::ElementPtr &_sdf,
  const physics::ModelPtr &_model,
  const common::Time &_currentTime)
  : FlashLightSetting(_sdf, _model, _currentTime),
  dataPtr(new LedSettingPrivate)
{
  // check if the visual element exists.
  this->dataPtr->visualExists = false;
  msgs::Link msg;
  this->Link()->FillMsg(msg);
  for (auto visualMsg : msg.visual())
  {
    if (visualMsg.name()
      == this->Link()->GetScopedName() + "::" + this->Name())
    {
      if (visualMsg.has_transparency())
      {
        this->dataPtr->transparency = visualMsg.transparency();
      }
      else
      {
        this->dataPtr->transparency = 0.2;
      }

      if (visualMsg.has_material()
        && visualMsg.material().has_emissive())
      {
        this->dataPtr->emissiveColor
          = msgs::Convert(visualMsg.material().emissive());
      }
      else
      {
        this->dataPtr->emissiveColor = ignition::math::Color(1, 1, 1);
      }

      this->dataPtr->visualExists = true;
    }
  }
}

//////////////////////////////////////////////////
LedSetting::~LedSetting()
{
}

//////////////////////////////////////////////////
void LedSetting::InitPubVisual(const transport::PublisherPtr &_pubVisual)
{
  // The PublisherPtr
  this->dataPtr->pubVisual = _pubVisual;

  if (this->dataPtr->visualExists)
  {
    // Make a message
    this->dataPtr->msg.set_name(
      this->Link()->GetScopedName() + "::" + this->Name());
    this->dataPtr->msg.set_parent_name(this->Link()->GetScopedName());
    uint32_t id;
    this->Link()->VisualId(this->Name(), id);
    this->dataPtr->msg.set_id(id);
    this->dataPtr->msg.set_transparency(this->dataPtr->transparency);
    msgs::Set(this->dataPtr->msg.mutable_material()->mutable_emissive(),
      this->dataPtr->emissiveColor);
  }
}

//////////////////////////////////////////////////
void LedSetting::Flash()
{
  // Call the function of the parent class.
  FlashLightSetting::Flash();

  // Make the appearance brighter.
  this->dataPtr->msg.set_transparency(0.0);
  msgs::Set(this->dataPtr->msg.mutable_material()->mutable_emissive(),
    this->dataPtr->emissiveColor);
  // Send the message.
  if (this->dataPtr->visualExists)
  {
    this->dataPtr->pubVisual->Publish(this->dataPtr->msg);
  }
}

//////////////////////////////////////////////////
void LedSetting::Dim()
{
  // Call the function of the parent class.
  FlashLightSetting::Dim();

  // Make the appearance darker.
  this->dataPtr->msg.set_transparency(this->dataPtr->transparency);
  msgs::Set(this->dataPtr->msg.mutable_material()->mutable_emissive(),
    ignition::math::Color(0, 0, 0));
  // Send the message.
  if (this->dataPtr->visualExists)
  {
    this->dataPtr->pubVisual->Publish(this->dataPtr->msg);
  }
}

//////////////////////////////////////////////////
LedPlugin::LedPlugin() : FlashLightPlugin(), dataPtr(new LedPluginPrivate)
{
  // Create a node
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();

  // advertise the topic to update lights
  this->dataPtr->pubVisual
    = this->dataPtr->node->Advertise<gazebo::msgs::Visual>("~/visual");

  this->dataPtr->pubVisual->WaitForConnection();
}

//////////////////////////////////////////////////
LedPlugin::~LedPlugin()
{
}

//////////////////////////////////////////////////
std::shared_ptr<FlashLightSetting> LedPlugin::CreateSetting(
  const sdf::ElementPtr &_sdf,
  const physics::ModelPtr &_model,
  const common::Time &_currentTime)
{
  return std::make_shared<LedSetting>(_sdf, _model, _currentTime);
}

//////////////////////////////////////////////////
void LedPlugin::InitSettingBySpecificData(
    std::shared_ptr<FlashLightSetting> &_setting)
{
  // Call the function of the parent class.
  FlashLightPlugin::InitSettingBySpecificData(_setting);

  std::dynamic_pointer_cast<LedSetting>(_setting)->InitPubVisual(
    this->dataPtr->pubVisual);
}
