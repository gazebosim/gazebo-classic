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

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"

#include "FlashLightPlugin.hh"

namespace gazebo
{
  /// \brief Internal data class to hold individual flash light settings.
  /// Settings for each flash light is separately stored in a
  /// FlashLightSettings class, which takes care of dynamic specifications
  /// such as duration and interval.
  class FlashLightSettings
  {
    /// \brief Constructor
    /// \param[in] _model The Model pointer holding the light to control.
    /// \param[in] _pubLight The publisher to send a message
    /// \param[in] _sdfFlashLight SDF data for flashlight settings.
    /// \param[in] _currentTime The current time point.
    public: FlashLightSettings(
      const physics::ModelPtr &_model,
      const transport::PublisherPtr &_pubLight,
      const sdf::ElementPtr &_sdfFlashLight,
      const common::Time &_currentTime);

    /// \brief Update the light based on the given time.
    /// \param[in] _currentTime The current point of time to update the lights.
    public: void UpdateLightInEnv(const common::Time &_currentTime);

    /// \brief Getter of name.
    /// \return The name of the light element.
    public: const std::string Name() const;

    /// \brief Getter of link.
    /// \return A pointer to the link element.
    public: const physics::LinkPtr Link() const;

    /// \brief Switch on (enable the flashlight).
    public: void SwitchOn();

    /// \brief Switch off (disable the flashlight).
    public: void SwitchOff();

    /// \brief Set the duration time.
    /// \param[in] _duration New duration time to set.
    public: void SetDuration(const double &_duration);

    /// \brief Set the interval time.
    /// \param[in] _interval New interval time to set.
    public: void SetInterval(const double &_interval);

    /// \brief The name of flash light.
    private: std::string name;

    /// \brief Link which holds this flash light.
    private: physics::LinkPtr link;

    /// \brief The time at which the current phase started.
    private: common::Time startTime;

    /// \brief The current switch state (the light itself is active or not).
    private: bool switchOn;

    /// \brief The current flasshing state (flash or dim).
    private: bool flashing;

    /// \brief The duration time to flash (in seconds).
    private: double duration;

    /// \brief The interval time between flashing (in seconds).
    /// When it is zero, the light is constant.
    private: double interval;

    /// \brief The length of the ray (in meters).
    private: double range;

    /// \brief The pointer to publisher to send a command to a light.
    private: transport::PublisherPtr pubLight;

    /// \brief Flash the light
    /// This function is internally used to update the light in the environment.
    private: void Flash();

    /// \brief Dim the light
    /// This function is internally used to update the light in the environment.
    private: void Dim();
  };

  class FlashLightPluginPrivate
  {
    /// \brief pointer to the model.
    public: physics::ModelPtr model;

    /// \brief pointer to the world.
    public: physics::WorldPtr world;

    /// \brief The pointer to node for communication.
    public: transport::NodePtr node;

    /// \brief The pointer to publisher to send a command to the light.
    public: transport::PublisherPtr pubLight;

    /// \brief The list of light settings to control.
    public: std::vector< std::shared_ptr<FlashLightSettings> > listFlashLight;

    /// \brief pointer to the update even connection.
    public: event::ConnectionPtr updateConnection;

    /// \brief Find a unit of settings by names.
    /// This is internally used to access an individual unit of light settings.
    /// If the link name is blank (""), the first match of the light name in the
    /// list will be returned.
    /// \param[in] _lightName The name of the light.
    /// \param[in] _linkName The name of the link holding the light.
    /// \return A pointer to the specified settings or nullptr if not found.
    public:
      std::shared_ptr<FlashLightSettings>
        SettingsByLightNameAndLinkName(
          const std::string &_lightName, const std::string &_linkName) const;
  };
}

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(FlashLightPlugin)

//////////////////////////////////////////////////
FlashLightSettings::FlashLightSettings(
  const physics::ModelPtr &_model,
  const transport::PublisherPtr &_pubLight,
  const sdf::ElementPtr &_sdfFlashLight,
  const common::Time &_currentTime)
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
  this->name = lightId.substr(posDelim+1, lightId.length());
  // link which holds this light
  this->link = _model->GetLink(lightId.substr(0, posDelim));
  // The PublisherPtr
  this->pubLight = _pubLight;
  // start time
  this->startTime = _currentTime;
  // current states
  this->switchOn = true;
  this->flashing = true;
  // duration
  if (_sdfFlashLight->HasElement("duration"))
  {
    this->duration = _sdfFlashLight->Get<double>("duration");
  }
  else
  {
    gzerr << "Parameter <duration> is missing." << std::endl;
  }
  // interval
  if (_sdfFlashLight->HasElement("interval"))
  {
    this->interval = _sdfFlashLight->Get<double>("interval");
  }
  else
  {
    gzerr << "Parameter <interval> is missing." << std::endl;
  }
  // range
  sdf::ElementPtr sdfLightInLink = this->link->GetSDF()->GetElement("light");
  while (sdfLightInLink)
  {
    if (sdfLightInLink->Get<std::string>("name") == this->name)
    {
      this->range
        = sdfLightInLink->GetElement("attenuation")->Get<double>("range");
      break;
    }
    sdfLightInLink = sdfLightInLink->GetNextElement("light");
  }

  // Initialize the light in the environment
  // Make a message
  msgs::Light msg;
  msg.set_name(this->link->GetScopedName() + "::" + this->name);
  msg.set_range(this->range);

  // Send the message to initialize the light
  this->pubLight->Publish(msg);
}

//////////////////////////////////////////////////
void FlashLightSettings::UpdateLightInEnv(const common::Time &_currentTime)
{
  // reset the time at the end of each phase
  if (_currentTime.Double() - this->startTime.Double()
        > this->duration + this->interval)
  {
    this->startTime = _currentTime;
  }

  if (this->switchOn)
  {
    // time to dim
    if (_currentTime.Double() - this->startTime.Double() > this->duration)
    {
      if (this->flashing)
      {
        this->Dim();
      }
    }
    // time to flash
    else
    {
      if (!this->flashing)
      {
        this->Flash();
      }
    }
  }
  else if (this->flashing)
  {
    this->Dim();
  }
}

//////////////////////////////////////////////////
const std::string FlashLightSettings::Name() const
{
  return this->name;
}

//////////////////////////////////////////////////
const physics::LinkPtr FlashLightSettings::Link() const
{
  return this->link;
}

//////////////////////////////////////////////////
void FlashLightSettings::SwitchOn()
{
  this->switchOn = true;
}

//////////////////////////////////////////////////
void FlashLightSettings::SwitchOff()
{
  this->switchOn = false;
}

//////////////////////////////////////////////////
void FlashLightSettings::SetDuration(const double &_duration)
{
  this->duration = _duration;
}

//////////////////////////////////////////////////
void FlashLightSettings::SetInterval(const double &_interval)
{
  this->interval = _interval;
}

//////////////////////////////////////////////////
void FlashLightSettings::Flash()
{
  // Make a message
  msgs::Light msg;
  msg.set_name(this->link->GetScopedName() + "::" + this->name);
  msg.set_range(this->range);
  // Send the message
  this->pubLight->Publish(msg);
  // Update the state
  this->flashing = true;
}

//////////////////////////////////////////////////
void FlashLightSettings::Dim()
{
  // Make a message
  msgs::Light msg;
  msg.set_name(this->link->GetScopedName() + "::" + this->name);
  msg.set_range(0.0);
  // Send the message
  this->pubLight->Publish(msg);
  // Update the state
  this->flashing = false;
}

//////////////////////////////////////////////////
std::shared_ptr<FlashLightSettings>
  FlashLightPluginPrivate::SettingsByLightNameAndLinkName(
  const std::string &_lightName, const std::string &_linkName) const
{
  std::shared_ptr<FlashLightSettings> ptr;

  std::vector< std::shared_ptr<FlashLightSettings> >::const_iterator it;
  for (it = this->listFlashLight.begin();
    it != this->listFlashLight.end(); ++it)
  {
    std::shared_ptr<FlashLightSettings> set
      = (std::shared_ptr<FlashLightSettings>)*it;

    if (set->Name() == _lightName)
    {
      if (_linkName.length() == 0
        || set->Link()->GetName() == _linkName)
      {
        ptr = set;
        break;
      }
    }
  }

  return ptr;
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
      std::shared_ptr<FlashLightSettings> settings
        = std::make_shared<FlashLightSettings>(this->dataPtr->model,
                                               this->dataPtr->pubLight,
                                               sdfFlashLight,
                                               currentTime);

      // Store the settings to the list
      this->dataPtr->listFlashLight.push_back(settings);
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

  std::vector< std::shared_ptr<FlashLightSettings> >::iterator it
    = this->dataPtr->listFlashLight.begin();

  while (it != this->dataPtr->listFlashLight.end())
  {
    std::shared_ptr<FlashLightSettings> set
      = (std::shared_ptr<FlashLightSettings>)*it;

    /// update the light
    set->UpdateLightInEnv(currentTime);

    ++it;
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
  bool found = false;
  std::shared_ptr<FlashLightSettings> set
    = this->dataPtr->SettingsByLightNameAndLinkName(_lightName, _linkName);

  if (set)
  {
    set->SwitchOn();
    found = true;
  }
  else
  {
    gzerr << "light: [" + _linkName + "/" + _lightName
     + "] does not exist." << std::endl;
  }

  return found;
}

//////////////////////////////////////////////////
bool FlashLightPlugin::TurnOnAll()
{
  bool found = false;

  std::vector< std::shared_ptr<FlashLightSettings> >::iterator it;
  for (it = this->dataPtr->listFlashLight.begin();
    it != this->dataPtr->listFlashLight.end(); ++it)
  {
    std::shared_ptr<FlashLightSettings> set
      = (std::shared_ptr<FlashLightSettings>)*it;

    set->SwitchOn();
    found = true;
  }

  if (!found)
    gzerr << "no flash lights exist to turn on." << std::endl;

  return found;
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
  bool found = false;
  std::shared_ptr<FlashLightSettings> set
    = this->dataPtr->SettingsByLightNameAndLinkName(_lightName, _linkName);

  if (set)
  {
    set->SwitchOff();
    found = true;
  }
  else
  {
    gzerr << "light: [" + _linkName + "/" + _lightName
     + "] does not exist." << std::endl;
  }

  return found;
}

//////////////////////////////////////////////////
bool FlashLightPlugin::TurnOffAll()
{
  bool found = false;

  std::vector< std::shared_ptr<FlashLightSettings> >::iterator it;
  for (it = this->dataPtr->listFlashLight.begin();
    it != this->dataPtr->listFlashLight.end(); ++it)
  {
    std::shared_ptr<FlashLightSettings> set
      = (std::shared_ptr<FlashLightSettings>)*it;

    set->SwitchOff();
    found = true;
  }

  if (!found)
    gzerr << "no flash lights exist to turn off." << std::endl;

  return found;
}

//////////////////////////////////////////////////
bool FlashLightPlugin::ChangeDuration(
  const std::string &_lightName, const std::string &_linkName,
  const double &_duration
)
{
  bool found = false;
  std::shared_ptr<FlashLightSettings> set
    = this->dataPtr->SettingsByLightNameAndLinkName(_lightName, _linkName);

  if (set)
  {
    set->SetDuration(_duration);
    found = true;
  }
  else
  {
    gzerr << "light <" + _lightName + "> does not exist." << std::endl;
  }

  return found;
}

//////////////////////////////////////////////////
bool FlashLightPlugin::ChangeInterval(
  const std::string &_lightName, const std::string &_linkName,
  const double &_interval
)
{
  bool found = false;
  std::shared_ptr<FlashLightSettings> set
    = this->dataPtr->SettingsByLightNameAndLinkName(_lightName, _linkName);

  if (set)
  {
    set->SetInterval(_interval);
    found = true;
  }
  else
  {
    gzerr << "light <" + _lightName + "> does not exist." << std::endl;
  }

  return found;
}
