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
#include "plugins/FlashLightPlugin.hh"

namespace gazebo
{
  struct Block
  {
    /// \brief The duration time to flash (in seconds).
    public: double duration;

    /// \brief The interval time between flashing (in seconds).
    /// When it is zero, the light is constant.
    public: double interval;

    /// \brief The color of light.
    public: ignition::math::Color color;
  };

  class FlashLightSettingPrivate
  {
    /// Constructor
    public: FlashLightSettingPrivate():
      switchOn(true), flashing(true), range(0),
      lightExists(false), currentBlockIndex(0)
    {
    }

    /// \brief Find the link holding the light to control.
    /// If multiple models are nested, this function is recursively called
    /// until the link is found.
    /// \param[in] _model A model to check.
    /// \param[in] _lightName the name of the light.
    /// \param[in] _linkName the name of the link.
    /// \return A pointer to the link. If not found, nullptr is returned.
    public: physics::LinkPtr FindLinkForLight(
      const physics::ModelPtr &_model,
      const std::string &_lightName, const std::string &_linkName)
    {
      auto childLink = _model->GetChildLink(_linkName);
      if (childLink && childLink->GetSDF()->HasElement("light"))
      {
        auto sdfLight = childLink->GetSDF()->GetElement("light");
        while (sdfLight)
        {
          if (sdfLight->Get<std::string>("name") == _lightName)
          {
            return childLink;
          }
          sdfLight = sdfLight->GetNextElement("light");
        }
      }
      for (auto model: _model->NestedModels())
      {
        auto foundLink = this->FindLinkForLight(model, _lightName, _linkName);
        if (foundLink)
        {
          return foundLink;
        }
      }

      return nullptr;
    }

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

    /// \brief The length of the ray (in meters).
    public: double range;

    /// \brief The pointer to publisher to send a command to a light.
    public: transport::PublisherPtr pubLight;

    /// \brief A message holding a flashlight command.
    public: msgs::Light msg;

    /// \brief True if <light> element exists.
    public: bool lightExists;

    /// \brief The list of blocks of light.
    public: std::vector< std::shared_ptr<Block> > blocks;

    /// \brief the index of the current block.
    public: int currentBlockIndex;
  };

  class FlashLightPluginPrivate
  {
    /// \brief Find a setting by names.
    /// This is internally used to access an individual setting.
    /// If the link name is blank (""), the first match of the light name in
    /// the list will be returned.
    /// \param[in] _lightName The name of the light.
    /// \param[in] _linkName The name of the link holding the light.
    /// \return A pointer to the specified setting or nullptr if not found.
    public: std::shared_ptr<FlashLightSetting>
      SettingByLightNameAndLinkName(
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

    /// \brief pointer to the model.
    public: physics::ModelPtr model;

    /// \brief pointer to the world.
    public: physics::WorldPtr world;

    /// \brief The pointer to node for communication.
    public: transport::NodePtr node;

    /// \brief The pointer to publisher to send a command to the light.
    public: transport::PublisherPtr pubLight;

    /// \brief The list of flashlight settings to control.
    public: std::vector< std::shared_ptr<FlashLightSetting> > listFlashLight;

    /// \brief pointer to the update even connection.
    public: event::ConnectionPtr updateConnection;
  };
}

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(FlashLightPlugin)

//////////////////////////////////////////////////
FlashLightSetting::FlashLightSetting(
  const sdf::ElementPtr &_sdf,
  const physics::ModelPtr &_model,
  const common::Time &_currentTime)
  : dataPtr(new FlashLightSettingPrivate)
{
  // Get the light name.
  std::string lightId;
  if (_sdf->HasElement("id"))
  {
    lightId = _sdf->Get<std::string>("id");
  }
  else
  {
    gzerr << "Parameter <id> is missing." << std::endl;
  }
  int posDelim = lightId.rfind("/");
  this->dataPtr->name = lightId.substr(posDelim+1, lightId.length());

  // link which holds this light
  this->dataPtr->link = this->dataPtr->FindLinkForLight(
                 _model, this->dataPtr->name,
                 lightId.substr(0, posDelim));

  if (_sdf->HasElement("block"))
  {
    sdf::ElementPtr sdfBlock = _sdf->GetElement("block");
    while (sdfBlock)
    {
      auto block = std::make_shared<Block>();
      // duration
      if (sdfBlock->HasElement("duration"))
      {
        block->duration = sdfBlock->Get<double>("duration");
      }
      else
      {
        gzerr << "Parameter <duration> is missing in a block." << std::endl;
      }
      // interval
      if (sdfBlock->HasElement("interval"))
      {
        block->interval = sdfBlock->Get<double>("interval");
      }
      else
      {
        gzerr << "Parameter <interval> is missing in a block." << std::endl;
      }
      // color
      if (sdfBlock->HasElement("color"))
      {
        block->color = sdfBlock->Get<ignition::math::Color>("color");
      }
      else
      {
        block->color.Reset();
      }

      this->dataPtr->blocks.push_back(block);
      sdfBlock = sdfBlock->GetNextElement("block");
    }
  }
  else
  {
    auto block = std::make_shared<Block>();
    // duration
    if (_sdf->HasElement("duration"))
    {
      block->duration = _sdf->Get<double>("duration");
    }
    else
    {
      gzerr << "Parameter <duration> is missing." << std::endl;
    }
    // interval
    if (_sdf->HasElement("interval"))
    {
      block->interval = _sdf->Get<double>("interval");
    }
    else
    {
      gzerr << "Parameter <interval> is missing." << std::endl;
    }
    // color
    if (_sdf->HasElement("color"))
    {
      block->color = _sdf->Get<ignition::math::Color>("color");
    }
    else
    {
      block->color.Reset();
    }

    this->dataPtr->blocks.push_back(block);
  }

  // start time
  this->dataPtr->startTime = _currentTime;

  // If link is not nullptr, the light exists.
  if (this->dataPtr->link)
  {
    // range
    if (this->dataPtr->link->GetSDF()->HasElement("light"))
    {
      auto sdfLight = this->dataPtr->link->GetSDF()->GetElement("light");
      while (sdfLight)
      {
        if (sdfLight->Get<std::string>("name") == this->dataPtr->name)
        {
          this->dataPtr->range
            = sdfLight->GetElement("attenuation")->Get<double>("range");
          break;
        }
        sdfLight = sdfLight->GetNextElement("light");
      }
      this->dataPtr->lightExists = true;
    }
  }
}

//////////////////////////////////////////////////
FlashLightSetting::~FlashLightSetting()
{
}

//////////////////////////////////////////////////
void FlashLightSetting::InitPubLight(
  const transport::PublisherPtr &_pubLight)
{
  // The PublisherPtr
  this->dataPtr->pubLight = _pubLight;

  if (this->dataPtr->lightExists)
  {
    // Make a message
    this->dataPtr->msg.set_name(
      this->dataPtr->link->GetScopedName() + "::" + this->dataPtr->name);
    this->dataPtr->msg.set_range(this->dataPtr->range);
  }
}

//////////////////////////////////////////////////
void FlashLightSetting::UpdateLightInEnv(const common::Time &_currentTime)
{
  int index = this->dataPtr->currentBlockIndex;

  // Reset the start time so the current time is within the current phase.
  if (_currentTime < this->dataPtr->startTime ||
      this->dataPtr->startTime
      + this->dataPtr->blocks[index]->duration
      + this->dataPtr->blocks[index]->interval <= _currentTime)
  {
    // initialize the start time.
    this->dataPtr->startTime = _currentTime;
    // proceed to the next block.
    index++;
    if (index >= static_cast<int>(this->dataPtr->blocks.size()))
    {
      index = 0;
    }
    this->dataPtr->currentBlockIndex = index;
  }

  if (this->dataPtr->switchOn)
  {
    // time to dim
    if (_currentTime - this->dataPtr->startTime
      > this->dataPtr->blocks[index]->duration)
    {
      if (this->dataPtr->flashing)
      {
        this->Dim();
      }
    }
    // time to flash
    else
    {
      // if there is more than one block, it calls Flash() at the beginning of
      // every block.
      if (this->dataPtr->blocks.size() > 1
        && this->dataPtr->startTime == _currentTime)
      {
        this->Flash();
      }
      // otherwise, it calls the function only if the light is not lit yet.
      else if (!this->dataPtr->flashing)
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
const std::string FlashLightSetting::Name() const
{
  return this->dataPtr->name;
}

//////////////////////////////////////////////////
const physics::LinkPtr FlashLightSetting::Link() const
{
  return this->dataPtr->link;
}

//////////////////////////////////////////////////
void FlashLightSetting::SwitchOn()
{
  this->dataPtr->switchOn = true;
}

//////////////////////////////////////////////////
void FlashLightSetting::SwitchOff()
{
  this->dataPtr->switchOn = false;
}

//////////////////////////////////////////////////
void FlashLightSetting::SetDuration(const double _duration, const int _index)
{
  if (0 <= _index && _index < static_cast<int>(this->dataPtr->blocks.size()))
  {
    this->dataPtr->blocks[_index]->duration = _duration;
  }
  else
  {
    gzerr << "The given index for block is out of range." << std::endl;
  }
}

//////////////////////////////////////////////////
void FlashLightSetting::SetDuration(const double _duration)
{
  for (auto block: this->dataPtr->blocks)
  {
    block->duration = _duration;
  }
}

//////////////////////////////////////////////////
void FlashLightSetting::SetInterval(const double _interval, const int _index)
{
  if (0 <= _index && _index < static_cast<int>(this->dataPtr->blocks.size()))
  {
    this->dataPtr->blocks[_index]->interval = _interval;
  }
  else
  {
    gzerr << "The given index for block is out of range." << std::endl;
  }
}

//////////////////////////////////////////////////
void FlashLightSetting::SetInterval(const double _interval)
{
  for (auto block: this->dataPtr->blocks)
  {
    block->interval = _interval;
  }
}

//////////////////////////////////////////////////
void FlashLightSetting::SetColor(
  const ignition::math::Color &_color, const int _index)
{
  if (0 <= _index && _index < static_cast<int>(this->dataPtr->blocks.size()))
  {
    this->dataPtr->blocks[_index]->color = _color;
  }
  else
  {
    gzerr << "The given index for block is out of range." << std::endl;
  }
}

//////////////////////////////////////////////////
void FlashLightSetting::SetColor(const ignition::math::Color &_color)
{
  for (auto block: this->dataPtr->blocks)
  {
    block->color = _color;
  }
}

//////////////////////////////////////////////////
unsigned int FlashLightSetting::BlockCount()
{
  return this->dataPtr->blocks.size();
}

//////////////////////////////////////////////////
bool FlashLightSetting::RemoveBlock(const int _index)
{
  if (_index < 0 || static_cast<int>(this->dataPtr->blocks.size()) <= _index)
  {
    return false;
  }

  this->dataPtr->blocks.erase(this->dataPtr->blocks.begin() + _index);

  return true;
}

//////////////////////////////////////////////////
void FlashLightSetting::InsertBlock(
  const double _duration, const double _interval,
  const ignition::math::Color &_color, const int _index)
{
  auto block = std::make_shared<Block>();

  block->duration = _duration;
  block->interval = _interval;
  block->color = _color;

  if (_index < 0 || static_cast<int>(this->dataPtr->blocks.size()) <= _index)
  {
    this->dataPtr->blocks.push_back(block);
  }
  else
  {
    this->dataPtr->blocks.insert(this->dataPtr->blocks.begin() + _index, block);
  }
}

//////////////////////////////////////////////////
void FlashLightSetting::Flash()
{
  // Set the range to the default value.
  this->dataPtr->msg.set_range(this->dataPtr->range);
  // set the color of light.
  if (this->dataPtr->blocks[this->dataPtr->currentBlockIndex]->color
    != ignition::math::Color::Black)
  {
    msgs::Set(this->dataPtr->msg.mutable_diffuse(),
      this->dataPtr->blocks[this->dataPtr->currentBlockIndex]->color);
    msgs::Set(this->dataPtr->msg.mutable_specular(),
      this->dataPtr->blocks[this->dataPtr->currentBlockIndex]->color);
  }
  // Send the message.
  if (this->dataPtr->lightExists)
  {
    this->dataPtr->pubLight->Publish(this->dataPtr->msg);
  }
  // Update the state.
  this->dataPtr->flashing = true;
}

//////////////////////////////////////////////////
void FlashLightSetting::Dim()
{
  // Set the range to zero.
  this->dataPtr->msg.set_range(0.0);
  // Send the message.
  if (this->dataPtr->lightExists)
  {
    this->dataPtr->pubLight->Publish(this->dataPtr->msg);
  }
  // Update the state.
  this->dataPtr->flashing = false;
}

//////////////////////////////////////////////////
ignition::math::Color FlashLightSetting::CurrentColor()
{
  return this->dataPtr->blocks[this->dataPtr->currentBlockIndex]->color;
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
  if (_sdf->HasElement("light"))
  {
    sdf::ElementPtr sdfFlashLight = _sdf->GetElement("light");
    while (sdfFlashLight)
    {
      // id required
      if (sdfFlashLight->HasElement("id"))
      {
        // Create an object of setting.
        std::shared_ptr<FlashLightSetting> setting
          = this->CreateSetting(
              sdfFlashLight, this->dataPtr->model, currentTime);

        // Initialize the object with the data specific to descendan classes.
        this->InitSettingBySpecificData(setting);

        // Store the setting to the list
        this->dataPtr->listFlashLight.push_back(setting);
      }
      else
      {
        // display an error message
        gzerr << "id does not exist in <light>" << std::endl;
      }

      sdfFlashLight = sdfFlashLight->GetNextElement("light");
    }
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
  if (_sdf->HasElement("light"))
  {
    sdf::ElementPtr sdfFlashLight = _sdf->GetElement("light");
    while (sdfFlashLight)
    {
      if (sdfFlashLight->HasElement("enable"))
      {
        std::string lightId = sdfFlashLight->Get<std::string>("id");
        int posDelim = lightId.rfind("/");
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

      sdfFlashLight = sdfFlashLight->GetNextElement("light");
    }
  }

  // listen to the update event by the World
  if (!this->dataPtr->listFlashLight.empty())
  {
    this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&FlashLightPlugin::OnUpdate, this));
  }
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
  const double _duration, const int _index
)
{
  std::shared_ptr<FlashLightSetting> setting
    = this->dataPtr->SettingByLightNameAndLinkName(_lightName, _linkName);

  if (setting)
  {
    if (_index >= 0)
    {
      setting->SetDuration(_duration, _index);
    }
    else
    {
      setting->SetDuration(_duration);
    }
    return true;
  }

  gzerr << "light <" + _lightName + "> does not exist." << std::endl;
  return false;
}

//////////////////////////////////////////////////
bool FlashLightPlugin::ChangeDuration(
  const std::string &_lightName, const std::string &_linkName,
  const double _duration
)
{
  return this->ChangeDuration(_lightName, _linkName, _duration, -1);
}

//////////////////////////////////////////////////
bool FlashLightPlugin::ChangeInterval(
  const std::string &_lightName, const std::string &_linkName,
  const double _interval, const int _index
)
{
  std::shared_ptr<FlashLightSetting> setting
    = this->dataPtr->SettingByLightNameAndLinkName(_lightName, _linkName);

  if (setting)
  {
    if (_index >= 0)
    {
      setting->SetInterval(_interval, _index);
    }
    else
    {
      setting->SetInterval(_interval);
    }
    return true;
  }

  gzerr << "light <" + _lightName + "> does not exist." << std::endl;
  return false;
}

//////////////////////////////////////////////////
bool FlashLightPlugin::ChangeInterval(
  const std::string &_lightName, const std::string &_linkName,
  const double _interval
)
{
  return this->ChangeInterval(_lightName, _linkName, _interval, -1);
}

//////////////////////////////////////////////////
bool FlashLightPlugin::ChangeColor(
  const std::string &_lightName, const std::string &_linkName,
  const ignition::math::Color &_color, const int _index
)
{
  std::shared_ptr<FlashLightSetting> setting
    = this->dataPtr->SettingByLightNameAndLinkName(_lightName, _linkName);

  if (setting)
  {
    if (_index >= 0)
    {
      setting->SetColor(_color, _index);
    }
    else
    {
      setting->SetColor(_color);
    }
    return true;
  }

  gzerr << "light <" + _lightName + "> does not exist." << std::endl;
  return false;
}

//////////////////////////////////////////////////
bool FlashLightPlugin::ChangeColor(
  const std::string &_lightName, const std::string &_linkName,
  const ignition::math::Color &_color
)
{
  return this->ChangeColor(_lightName, _linkName, _color, -1);
}

//////////////////////////////////////////////////
std::shared_ptr<FlashLightSetting>
  FlashLightPlugin::CreateSetting(
    const sdf::ElementPtr &_sdf,
    const physics::ModelPtr &_model,
    const common::Time &_currentTime)
{
  return std::make_shared<FlashLightSetting>(_sdf, _model, _currentTime);
}

//////////////////////////////////////////////////
void FlashLightPlugin::InitSettingBySpecificData(
    std::shared_ptr<FlashLightSetting> &_setting)
{
  _setting->InitPubLight(this->dataPtr->pubLight);
}
