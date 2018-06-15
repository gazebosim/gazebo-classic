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

#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "plugins/FlashLightPlugin.hh"

namespace gazebo
{
  class LEDLightPlugin::LEDSettingPrivate
  {
    /// \brief The pointer to publisher to send a command to update a visual.
    public: transport::PublisherPtr pubVisual;

    /// \brief A message holding a Visual message.
    public: msgs::Visual msg;
  };

  class LEDPluginPrivate
  {
    /// \brief The pointer to publisher to send a command to the visual.
    public: transport::PublisherPtr pubVisual;
  };
}

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(LEDPlugin)

//////////////////////////////////////////////////
LEDPlugin::LEDSetting::LEDSetting(
    const physics::ModelPtr &_model,
    const transport::PublisherPtr &_pubLight,
    const sdf::ElementPtr &_sdfFlashLight,
    const common::Time &_currentTime
    const transport::PublisherPtr &_pubVisual):
  FlashLightSetting(_mode, _pubLight, _sdfFlashLight, _currentTime),
  dataPtr(new LEDSettingPrivate)
{
  // The PublisherPtr
  this->dataPtr->pubVisual = _pubVisual;

  // Initialize the light in the environment
  // Make a message
  this->dataPtr->msg.set_name(this->GetLink()->GetScopedName() + "::" + this->GetName());
  //this->dataPtr->msg.set_parent_name(this->GetLin()->GetScopedName());
  this->dataPtr->msg.set_transparency(1.0);

  // Send the message to initialize the light
  this->dataPtr->pubVisual->Publish(this->dataPtr->msg);
}

//////////////////////////////////////////////////
LEDPlugin::LEDSetting::~LEDSetting()
{
}

//////////////////////////////////////////////////
void LEDPlugin::LEDSetting::Flash()
{
  // Call the function of the parent class.
  FlashLightSetting::Flash();

  // Make the appearance brighter.
  this->dataPtr->msg.set_transparency(0.0);
  // Send the message.
  this->dataPtr->pubVisual->Publish(this->dataPtr->msg);
}

//////////////////////////////////////////////////
void LEDPlugin::LEDSetting::Dim()
{
  // Call the function of the parent class.
  FlashLightSetting::Dim();

  // Make the appearance darker.
  this->dataPtr->msg.set_transparency(1.0);
  // Send the message.
  this->dataPtr->pubVisual->Publish(this->dataPtr->msg);
}

//////////////////////////////////////////////////
LEDPlugin::LEDPlugin() : FlashLightPlugin()
{
  // advertise the topic to update lights
  this->dataPtr->pubLight
    = this->dataPtr->node->Advertise<gazebo::msgs::Light>("~/light/modify");

  this->dataPtr->pubLight->WaitForConnection();
}

//////////////////////////////////////////////////
LEDPlugin::~LEDPlugin()
{
}

//////////////////////////////////////////////////
std::shared_ptr<FlashLightSetting> CreateSetting(
  const sdf::ElementPtr &_sdfFlashLight,
  const common::Time &_currentTime)
{
  return dynamic_cast< shared_ptr<FlashLightSetting> >(
    std::make_shared<LEDSetting>(
      this->GetModel, _pubLight, _sdfFlashLight, _currentTime));
}
