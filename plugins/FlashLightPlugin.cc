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

#include "plugins/FlashLightPlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(FlashLightPlugin)

// static members
transport::NodePtr FlashLightSettings::node;
transport::PublisherPtr FlashLightSettings::pubLight;

//////////////////////////////////////////////////
void FlashLightSettings::Flash()
{
  // Make a message
  msgs::Light msg;
  msg.set_name(this->link->GetScopedName() + "::" + this->name);
  msg.set_range(this->range);
  // Send the message
  FlashLightSettings::pubLight->Publish(msg);
  // Update the state
  this->f_flashing = true;
}

//////////////////////////////////////////////////
void FlashLightSettings::Dim()
{
  // Make a message
  msgs::Light msg;
  msg.set_name(this->link->GetScopedName() + "::" + this->name);
  msg.set_range(0.0);
  // Send the message
  FlashLightSettings::pubLight->Publish(msg);
  // Update the state
  this->f_flashing = false;
}

//////////////////////////////////////////////////
FlashLightSettings::FlashLightSettings(
  const physics::ModelPtr &_model,
  const sdf::ElementPtr &_sdfFlashLight,
  const common::Time &_current_time)
{
  if (!FlashLightSettings::node)
  {
    // Create a node
    FlashLightSettings::node = transport::NodePtr(new transport::Node());
    FlashLightSettings::node->Init();

    // advertise the topic to update lights
    FlashLightSettings::pubLight =
      FlashLightSettings::node
      ->Advertise<gazebo::msgs::Light>("~/light/modify");

    FlashLightSettings::pubLight->WaitForConnection();
  }

  // name
  std::string light_id = _sdfFlashLight->Get<std::string>("light_id");
  int pos_delim = light_id.find("/");
  this->name = light_id.substr(pos_delim+1, light_id.length());
  // link which holds this light
  this->link = _model->GetLink(light_id.substr(0, pos_delim));
  // start time
  this->start_time = _current_time;
  // current states
  this->f_switch_on = true;
  this->f_flashing = true;
  // duration
  if (_sdfFlashLight->HasElement("duration"))
  {
    this->duration = _sdfFlashLight->Get<double>("duration");
  }
  else
  {
    this->interval = 0.5;
  }
  // interval
  if (_sdfFlashLight->HasElement("interval"))
  {
    this->interval = _sdfFlashLight->Get<double>("interval");
  }
  else
  {
    this->interval = 0.5;
  }
  // range
  if (_sdfFlashLight->HasElement("range"))
  {
    this->range = _sdfFlashLight->Get<double>("range");
  }
  else
  {
    this->range = 30.0;
  }

  // Initialize the light in the environment
  // Make a message
  msgs::Light msg;
  msg.set_name(this->link->GetScopedName() + "::" + this->name);
  msg.set_range(this->range);

  // Send the message to initialize the light
  FlashLightSettings::pubLight->Publish(msg);
}

//////////////////////////////////////////////////
void FlashLightSettings::UpdateLightInEnv(const common::Time &_current_time)
{
  // reset the time at the and of each phase
  if (_current_time.Double() - this->start_time.Double()
        > this->duration + this->interval)
  {
    this->start_time = _current_time;
  }

  if (this->f_switch_on)
  {
    // time to dim
    if (_current_time.Double() - this->start_time.Double()
          > this->duration)
    {
      if (this->f_flashing == true)
      {
        this->Dim();
      }
    }
    // time to flash
    else
    {
      if (this->f_flashing == false)
      {
        this->Flash();
      }
    }
  }
  else if (this->f_flashing)
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
  this->f_switch_on = true;
}

//////////////////////////////////////////////////
void FlashLightSettings::SwitchOff()
{
  this->f_switch_on = false;
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
void FlashLightPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Store the pointers to the model and world
  this->model = _parent;
  this->world = _parent->GetWorld();

  // Get the current time
  common::Time current_time = this->world->SimTime();

  // Get the parameters from sdf
  sdf::ElementPtr sdfFlashLight = _sdf->GetElement("flash_light");
  while (sdfFlashLight)
  {
    // light_id required
    if (sdfFlashLight->HasElement("light_id"))
    {
      // Get the setting information from sdf
      std::shared_ptr<FlashLightSettings> settings
        = std::make_shared<FlashLightSettings>(this->model,
                                               sdfFlashLight,
                                               current_time);

      // Store the settings to the list
      this->list_flash_light.push_back(settings);
    }
    else
    {
      // display an error message
      gzerr << "no name field exists in <flash_light>" << std::endl;
    }

    sdfFlashLight = sdfFlashLight->GetNextElement("flash_light");
  }

  // Turn on/off the lights if <start> element is given
  if (_sdf->HasElement("start"))
  {
    if (_sdf->Get<bool>("start") == true)
    {
      this->TurnOnAll();
    }
    else
    {
      this->TurnOffAll();
    }
  }
  sdfFlashLight = _sdf->GetElement("flash_light");
  while (sdfFlashLight)
  {
    // light_id required
    if (sdfFlashLight->HasElement("start"))
    {
      std::string light_id = sdfFlashLight->Get<std::string>("light_id");
      int pos_delim = light_id.find("/");
      std::string light_name = light_id.substr(pos_delim+1, light_id.length());
      std::string link_name = light_id.substr(0, pos_delim);
      if (sdfFlashLight->Get<bool>("start") == true)
      {
        this->TurnOn(light_name, link_name);
      }
      else
      {
        this->TurnOff(light_name, link_name);
      }
    }

    sdfFlashLight = sdfFlashLight->GetNextElement("flash_light");
  }

  // listen to the update event by the World
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&FlashLightPlugin::OnUpdate, this));
}

//////////////////////////////////////////////////
void FlashLightPlugin::OnUpdate()
{
  common::Time current_time = this->world->SimTime();

  std::vector< std::shared_ptr<FlashLightSettings> >::iterator it
    = this->list_flash_light.begin();

  while (it != this->list_flash_light.end())
  {
    std::shared_ptr<FlashLightSettings> set
      = (std::shared_ptr<FlashLightSettings>)*it;

    /// update the light
    set->UpdateLightInEnv(current_time);

    ++it;
  }
}

//////////////////////////////////////////////////
bool FlashLightPlugin::TurnOn(const std::string &_light_name)
{
  return this->TurnOn(_light_name, "");
}

//////////////////////////////////////////////////
bool FlashLightPlugin::TurnOn(
  const std::string &_light_name, const std::string &_link_name)
{
  bool f_found = false;
  std::shared_ptr<FlashLightSettings> set
    = this->FindSettings(_light_name, _link_name);

  if (set != NULL)
  {
    set->SwitchOn();
    f_found = true;
  }
  else
  {
    gzerr << "light: [" + _link_name + "/" + _light_name
     + "] does not exist." << std::endl;
  }

  return f_found;
}

//////////////////////////////////////////////////
bool FlashLightPlugin::TurnOnAll()
{
  bool f_found = false;

  std::vector< std::shared_ptr<FlashLightSettings> >::iterator it;
  for (it = this->list_flash_light.begin();
    it != this->list_flash_light.end(); ++it)
  {
    std::shared_ptr<FlashLightSettings> set
      = (std::shared_ptr<FlashLightSettings>)*it;

    set->SwitchOn();
    f_found = true;
  }

  if (f_found == false)
    gzerr << "no flash lights exist to turn on." << std::endl;

  return f_found;
}

//////////////////////////////////////////////////
bool FlashLightPlugin::TurnOff(const std::string &_light_name)
{
  return this->TurnOff(_light_name, "");
}

//////////////////////////////////////////////////
bool FlashLightPlugin::TurnOff(const std::string &_light_name,
  const std::string &_link_name)
{
  bool f_found = false;
  std::shared_ptr<FlashLightSettings> set
    = this->FindSettings(_light_name, _link_name);

  if (set != NULL)
  {
    set->SwitchOff();
    f_found = true;
  }
  else
  {
    gzerr << "light: [" + _link_name + "/" + _light_name
     + "] does not exist." << std::endl;
  }

  return f_found;
}

//////////////////////////////////////////////////
bool FlashLightPlugin::TurnOffAll()
{
  bool f_found = false;

  std::vector< std::shared_ptr<FlashLightSettings> >::iterator it;
  for (it = this->list_flash_light.begin();
    it != this->list_flash_light.end(); ++it)
  {
    std::shared_ptr<FlashLightSettings> set
      = (std::shared_ptr<FlashLightSettings>)*it;

    set->SwitchOff();
    f_found = true;
  }

  if (f_found == false)
    gzerr << "no flash lights exist to turn off." << std::endl;

  return f_found;
}

//////////////////////////////////////////////////
bool FlashLightPlugin::ChangeDuration(
  const std::string &_light_name, const std::string &_link_name,
  const double &_duration
)
{
  bool f_found = false;
  std::shared_ptr<FlashLightSettings> set
    = this->FindSettings(_light_name, _link_name);

  if (set != NULL)
  {
    set->SetDuration(_duration);
  }
  else
  {
    gzerr << "light <" + _light_name + "> does not exist." << std::endl;
  }

  return f_found;
}

//////////////////////////////////////////////////
bool FlashLightPlugin::ChangeInterval(
  const std::string &_light_name, const std::string &_link_name,
  const double &_interval
)
{
  bool f_found = false;
  std::shared_ptr<FlashLightSettings> set
    = this->FindSettings(_light_name, _link_name);

  if (set != NULL)
  {
    set->SetInterval(_interval);
  }
  else
  {
    gzerr << "light <" + _light_name + "> does not exist." << std::endl;
  }

  return f_found;
}

//////////////////////////////////////////////////
std::shared_ptr<FlashLightSettings> FlashLightPlugin::FindSettings(
  const std::string &_light_name, const std::string &_link_name
)
{
  std::shared_ptr<FlashLightSettings> ptr;

  std::vector< std::shared_ptr<FlashLightSettings> >::iterator it;
  for (it = this->list_flash_light.begin();
    it != this->list_flash_light.end(); ++it)
  {
    std::shared_ptr<FlashLightSettings> set
      = (std::shared_ptr<FlashLightSettings>)*it;

    if (set->Name() == _light_name)
    {
      if (_link_name.length() == 0
        || set->Link()->GetName() == _link_name)
      {
        ptr = set;
      }
    }
  }

  return ptr;
}
