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
    /// \brief Constructor.
    public: LedSettingPrivate():
      transparency(0.2), defaultEmissiveColor(ignition::math::Color::White),
      visualExists(false)
    {
    }

    /// \brief The transparency when the ligth is off.
    public: double transparency;

    /// \brief The emissive color.
    public: ignition::math::Color defaultEmissiveColor;

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

      if (visualMsg.has_material()
        && visualMsg.material().has_emissive())
      {
        this->dataPtr->defaultEmissiveColor
          = msgs::Convert(visualMsg.material().emissive());
      }

      this->dataPtr->visualExists = true;
      break;
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
  }
}

//////////////////////////////////////////////////
void LedSetting::Flash()
{
  // Call the function of the parent class.
  FlashLightSetting::Flash();

  // Make the appearance brighter.
  this->dataPtr->msg.set_transparency(0.0);
  ignition::math::Color color = this->CurrentColor();
  if (color != ignition::math::Color::Black)
  {
    // If the base class is using a specific color rather than the default,
    // apply it to the visual object.
    msgs::Set(this->dataPtr->msg.mutable_material()->mutable_diffuse(), color);
    msgs::Set(this->dataPtr->msg.mutable_material()->mutable_emissive(), color);
    msgs::Set(this->dataPtr->msg.mutable_material()->mutable_specular(), color);
    msgs::Set(this->dataPtr->msg.mutable_material()->mutable_ambient(), color);
  }
  else
  {
    // Otherwise, just apply the default color.
    msgs::Set(this->dataPtr->msg.mutable_material()->mutable_emissive(),
      this->dataPtr->defaultEmissiveColor);
  }

  // Send the message.
  if (this->dataPtr->visualExists)
  {
    // NOTE: this if statement is to make sure that a visual object to update
    // has been created in the environment before publishing a message.
    // Otherwise, a duplicate object will be created and the original one will
    // never be updated.
    // This problem is solved by the patch (Pull Request # 2983), which has
    // been merged into gazebo7 as of July 16, 2018. This if satement should be
    // removed once the patch is forwarded up to gazebo9.
    if (this->Link()->GetWorld()->SimTime() > 2.0)
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
    // NOTE: this if statement is to make sure that a visual object to update
    // has been created in the environment before publishing a message.
    // Otherwise, a duplicate object will be created and the original one will
    // never be updated.
    // This problem is solved by the patch (Pull Request # 2983), which has
    // been merged into gazebo7 as of July 16, 2018. This if satement should be
    // removed once the patch is forwarded up to gazebo9.
    if (this->Link()->GetWorld()->SimTime() > 2.0)
      this->dataPtr->pubVisual->Publish(this->dataPtr->msg);
  }
}

//////////////////////////////////////////////////
LedPlugin::LedPlugin() : FlashLightPlugin(), dataPtr(new LedPluginPrivate)
{
  // Create a node
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();

  // Advertise the topic to update lights
  this->dataPtr->pubVisual
    = this->dataPtr->node->Advertise<gazebo::msgs::Visual>("~/visual");
  // NOTE: it should not call WaitForConnection() since there could be no
  // subscriber to "~/visual" topic if the render engine is not running (e.g.,
  // rostest), leading to a hang-up.
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
