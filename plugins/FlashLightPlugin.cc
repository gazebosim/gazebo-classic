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
  class FlashLightPlugin::FlashLightSettingPrivate
  {
    /// \brief The name of flash light.
    public: std::string name;

    /// \brief Link which holds this flash light.
    public: physics::LinkPtr link;

    /// \brief The time at which the current phase started.
    public: common::Time startTime;

    /// \brief The current switch state (the light itself is active or not).
    public: bool switchOn;

    /// \brief The current flasshing state (flash or dim).
    public: bool flashing;

    /// \brief The duration time to flash (in seconds).
    public: double duration;

    /// \brief The interval time between flashing (in seconds).
    /// When it is zero, the light is constant.
    public: double interval;

    /// \brief The length of the ray (in meters).
    public: double range;

    /// \brief The pointer to publisher to send a command to a light.
    public: transport::PublisherPtr pubLight;

    /// \brief A message holding a flashlight command.
    public: msgs::Light msg;
  };

  class FlashLightPluginPrivate
  {
    /// \brief Find a setting by names.
    /// This is internally used to access an individual setting.
    /// If the link name is blank (""), the first match of the light name in the
    /// list will be returned.
    /// \param[in] _lightName The name of the light.
    /// \param[in] _linkName The name of the link holding the light.
    /// \return A pointer to the specified setting or nullptr if not found.
    public:
      std::shared_ptr<FlashLightPlugin::FlashLightSetting>
        SettingByLightNameAndLinkName(
          const std::string &_lightName, const std::string &_linkName) const;

    /// \brief pointer to the model.
    public: physics::ModelPtr model;

    /// \brief pointer to the world.
    public: physics::WorldPtr world;

    /// \brief The pointer to node for communication.
    public: transport::NodePtr node;

    /// \brief The pointer to publisher to send a command to the light.
    public: transport::PublisherPtr pubLight;

    /// \brief The list of flashlight settings to control.
    public: std::vector< std::shared_ptr<FlashLightPlugin::FlashLightSetting> > listFlashLight;

    /// \brief pointer to the update even connection.
    public: event::ConnectionPtr updateConnection;
  };
}

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(FlashLightPlugin)

//////////////////////////////////////////////////
FlashLightPlugin::FlashLightSetting::FlashLightSetting(
    const physics::ModelPtr &_model,
    const transport::PublisherPtr &_pubLight,
    const sdf::ElementPtr &_sdfFlashLight,
    const common::Time &_currentTime):
  dataPtr(new FlashLightPlugin::FlashLightSettingPrivate)
{
  // name
  std::string lightId;
  if (_sdfFlashLight->HasElement("light_id"))
  {
    lightId = _sdfFlashLight->Get<std::string>("light_id");
  }
  else
  {
    gzerr << "Parameter <light_id> is missing." << std::endl;
  }
  int posDelim = lightId.find("/");
  this->dataPtr->name = lightId.substr(posDelim+1, lightId.length());
  // link which holds this light
  this->dataPtr->link = _model->GetLink(lightId.substr(0, posDelim));
  // The PublisherPtr
  this->dataPtr->pubLight = _pubLight;
  // start time
  this->dataPtr->startTime = _currentTime;
  // current states
  this->dataPtr->switchOn = true;
  this->dataPtr->flashing = true;
  // duration
  if (_sdfFlashLight->HasElement("duration"))
  {
    this->dataPtr->duration = _sdfFlashLight->Get<double>("duration");
  }
  else
  {
    gzerr << "Parameter <duration> is missing." << std::endl;
  }
  // interval
  if (_sdfFlashLight->HasElement("interval"))
  {
    this->dataPtr->interval = _sdfFlashLight->Get<double>("interval");
  }
  else
  {
    gzerr << "Parameter <interval> is missing." << std::endl;
  }
  // range
  sdf::ElementPtr sdfLightInLink = this->dataPtr->link->GetSDF()->GetElement("light");
  while (sdfLightInLink)
  {
    if (sdfLightInLink->Get<std::string>("name") == this->dataPtr->name)
    {
      this->dataPtr->range
        = sdfLightInLink->GetElement("attenuation")->Get<double>("range");
      break;
    }
    sdfLightInLink = sdfLightInLink->GetNextElement("light");
  }

  // Initialize the light in the environment
  // Make a message
  this->dataPtr->msg.set_name(this->dataPtr->link->GetScopedName() + "::" + this->dataPtr->name);
  this->dataPtr->msg.set_range(this->dataPtr->range);

  // Send the message to initialize the light
  this->dataPtr->pubLight->Publish(this->dataPtr->msg);
}

//////////////////////////////////////////////////
void FlashLightPlugin::FlashLightSetting::UpdateLightInEnv(const common::Time &_currentTime)
{
  // Reset the start time so the current time is within the current phase.
  if (_currentTime < this->dataPtr->startTime ||
      this->dataPtr->startTime + this->dataPtr->duration + this->dataPtr->interval <= _currentTime)
  {
    this->dataPtr->startTime = _currentTime;
  }

  if (this->dataPtr->switchOn)
  {
    // time to dim
    if (_currentTime - this->dataPtr->startTime > this->dataPtr->duration)
    {
      if (this->dataPtr->flashing)
      {
        this->Dim();
      }
    }
    // time to flash
    else
    {
      if (!this->dataPtr->flashing)
      {
        this->Flash();
      }
    }
  }
  else if (this->dataPtr->flashing)
  {
    this->Dim();
  }
}

//////////////////////////////////////////////////
const std::string FlashLightPlugin::FlashLightSetting::Name() const
{
  return this->dataPtr->name;
}

//////////////////////////////////////////////////
const physics::LinkPtr FlashLightPlugin::FlashLightSetting::Link() const
{
  return this->dataPtr->link;
}

//////////////////////////////////////////////////
void FlashLightPlugin::FlashLightSetting::SwitchOn()
{
  this->dataPtr->switchOn = true;
}

//////////////////////////////////////////////////
void FlashLightPlugin::FlashLightSetting::SwitchOff()
{
  this->dataPtr->switchOn = false;
}

//////////////////////////////////////////////////
void FlashLightPlugin::FlashLightSetting::SetDuration(const double &_duration)
{
  this->dataPtr->duration = _duration;
}

//////////////////////////////////////////////////
void FlashLightPlugin::FlashLightSetting::SetInterval(const double &_interval)
{
  this->dataPtr->interval = _interval;
}

//////////////////////////////////////////////////
void FlashLightPlugin::FlashLightSetting::Flash()
{
  // Set the range to the default value.
  this->dataPtr->msg.set_range(this->dataPtr->range);
  // Send the message.
  this->dataPtr->pubLight->Publish(this->dataPtr->msg);
  // Update the state.
  this->dataPtr->flashing = true;
}

//////////////////////////////////////////////////
void FlashLightPlugin::FlashLightSetting::Dim()
{
  // Set the range to zero.
  this->dataPtr->msg.set_range(0.0);
  // Send the message.
  this->dataPtr->pubLight->Publish(this->dataPtr->msg);
  // Update the state.
  this->dataPtr->flashing = false;
}

//////////////////////////////////////////////////
std::shared_ptr<FlashLightPlugin::FlashLightSetting>
  FlashLightPluginPrivate::SettingByLightNameAndLinkName(
  const std::string &_lightName, const std::string &_linkName) const
{
  for (auto &setting: this->listFlashLight)
  {
    if (setting->Name() == _lightName)
    {
      if (_linkName.length() == 0
        || setting->Link()->GetName() == _linkName)
      {
        return setting;
      }
    }
  }

  return nullptr;
}

//////////////////////////////////////////////////
FlashLightPlugin::FlashLightPlugin() : ModelPlugin(),
  dataPtr(new FlashLightPluginPrivate)
{
  // Create a node
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();

  // advertise the topic to update lights
  this->dataPtr->pubLight
    = this->dataPtr->node->Advertise<gazebo::msgs::Light>("~/light/modify");

  this->dataPtr->pubLight->WaitForConnection();
}

//////////////////////////////////////////////////
FlashLightPlugin::~FlashLightPlugin()
{
}

//////////////////////////////////////////////////
void FlashLightPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  gzdbg << "test3" << std::endl;
  // Store the pointers to the model and world
  this->dataPtr->model = _parent;
  this->dataPtr->world = _parent->GetWorld();

  // Get the current time
  common::Time currentTime = this->dataPtr->world->SimTime();

  // Get the parameters from sdf
  sdf::ElementPtr sdfFlashLight = _sdf->GetElement("flash_light");
  while (sdfFlashLight)
  {
    // light_id required
    if (sdfFlashLight->HasElement("light_id"))
    {
      // Get the setting information from sdf
      std::shared_ptr<FlashLightSetting> setting
        = std::make_shared<FlashLightSetting>(this->dataPtr->model,
                                               this->dataPtr->pubLight,
                                               sdfFlashLight,
                                               currentTime);

      // Store the setting to the list
      this->dataPtr->listFlashLight.push_back(setting);
    }
    else
    {
      // display an error message
      gzerr << "no name field exists in <flash_light>" << std::endl;
    }

    sdfFlashLight = sdfFlashLight->GetNextElement("flash_light");
  }

  // Turn on/off all the lights if <enable> element is given
  if (_sdf->HasElement("enable"))
  {
    if (_sdf->Get<bool>("enable"))
    {
      this->TurnOnAll();
    }
    else
    {
      this->TurnOffAll();
    }
  }
  // Turn on/off a specific light if <enable> is specifically given.
  sdfFlashLight = _sdf->GetElement("flash_light");
  while (sdfFlashLight)
  {
    // light_id required
    if (sdfFlashLight->HasElement("enable"))
    {
      std::string lightId = sdfFlashLight->Get<std::string>("light_id");
      int posDelim = lightId.find("/");
      std::string lightName = lightId.substr(posDelim+1, lightId.length());
      std::string linkName = lightId.substr(0, posDelim);
      if (sdfFlashLight->Get<bool>("enable"))
      {
        this->TurnOn(lightName, linkName);
      }
      else
      {
        this->TurnOff(lightName, linkName);
      }
    }

    sdfFlashLight = sdfFlashLight->GetNextElement("flash_light");
  }

  // listen to the update event by the World
  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&FlashLightPlugin::OnUpdate, this));
}

//////////////////////////////////////////////////
void FlashLightPlugin::OnUpdate()
{
  common::Time currentTime = this->dataPtr->world->SimTime();

  for (auto &setting: this->dataPtr->listFlashLight)
  {
    /// update the light
    setting->UpdateLightInEnv(currentTime);
  }
}

//////////////////////////////////////////////////
bool FlashLightPlugin::TurnOn(const std::string &_lightName)
{
  return this->TurnOn(_lightName, "");
}

//////////////////////////////////////////////////
bool FlashLightPlugin::TurnOn(
  const std::string &_lightName, const std::string &_linkName)
{
  std::shared_ptr<FlashLightSetting> setting
    = this->dataPtr->SettingByLightNameAndLinkName(_lightName, _linkName);

  if (setting)
  {
    setting->SwitchOn();
    return true;
  }

  gzerr << "light: [" + _linkName + "/" + _lightName + "] does not exist."
        << std::endl;
  return false;
}

//////////////////////////////////////////////////
bool FlashLightPlugin::TurnOnAll()
{
  if (this->dataPtr->listFlashLight.empty())
  {
    gzerr << "no flash lights exist to turn on." << std::endl;
    return false;
  }

  for (auto &setting: this->dataPtr->listFlashLight)
  {
    setting->SwitchOn();
  }

  return true;
}

//////////////////////////////////////////////////
bool FlashLightPlugin::TurnOff(const std::string &_lightName)
{
  return this->TurnOff(_lightName, "");
}

//////////////////////////////////////////////////
bool FlashLightPlugin::TurnOff(const std::string &_lightName,
  const std::string &_linkName)
{
  std::shared_ptr<FlashLightSetting> setting
    = this->dataPtr->SettingByLightNameAndLinkName(_lightName, _linkName);

  if (setting)
  {
    setting->SwitchOff();
    return true;
  }

  gzerr << "light: [" + _linkName + "/" + _lightName + "] does not exist."
        << std::endl;
  return false;
}

//////////////////////////////////////////////////
bool FlashLightPlugin::TurnOffAll()
{
  if (this->dataPtr->listFlashLight.empty())
  {
    gzerr << "no flash lights exist to turn off." << std::endl;
    return false;
  }

  for (auto &setting: this->dataPtr->listFlashLight)
  {
    setting->SwitchOff();
  }

  return true;
}

//////////////////////////////////////////////////
bool FlashLightPlugin::ChangeDuration(
  const std::string &_lightName, const std::string &_linkName,
  const double &_duration
)
{
  std::shared_ptr<FlashLightSetting> setting
    = this->dataPtr->SettingByLightNameAndLinkName(_lightName, _linkName);

  if (setting)
  {
    setting->SetDuration(_duration);
    return true;
  }

  gzerr << "light <" + _lightName + "> does not exist." << std::endl;
  return false;
}

//////////////////////////////////////////////////
bool FlashLightPlugin::ChangeInterval(
  const std::string &_lightName, const std::string &_linkName,
  const double &_interval
)
{
  std::shared_ptr<FlashLightSetting> setting
    = this->dataPtr->SettingByLightNameAndLinkName(_lightName, _linkName);

  if (setting)
  {
    setting->SetInterval(_interval);
    return true;
  }

  gzerr << "light <" + _lightName + "> does not exist." << std::endl;
  return false;
}
