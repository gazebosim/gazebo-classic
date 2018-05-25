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

#ifndef GAZEBO_PLUGINS_FLASHLIGHTPLUGIN_HH_
#define GAZEBO_PLUGINS_FLASHLIGHTPLUGIN_HH_

#include <string>
#include <vector>
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"

namespace gazebo
{
  // forward declaration
  class FlashLightSettings;

  /// \brief A plugin that turns on/off a light component in the model.
  // This plugin accesses <light> components in the model specified
  // by <flash_light> as a parameter.
  // The light is specified by <light_id>, including link and light names
  // separated by a slash "/"
  //
  // <flash_light>
  //  <light_id>link_light/light_source</light_id>
  //  <duration>0.1</duration>
  //  <interval>0.4</interval>
  // </flash_light>
  //
  // More than one <flash_light> can exist.
  //
  // Settings for each flash light is separately stored in a
  // FlashLightSettings class, which takes care of dynamic specifications
  // such as duration and interval.
  //
  // This base class provides basic functions to turn the lights on/off.
  // Users can create their own flash light plugin by inheriting this base
  // model.
  //
  class GAZEBO_VISIBLE FlashLightPlugin : public ModelPlugin
  {
    /// \brief pointer to the model
    protected: physics::ModelPtr model;

    /// \brief pointer to the world
    protected: physics::WorldPtr world;

    /// \brief Called when the plugin is loaded
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Called by the world update start event
    public: virtual void OnUpdate();

    /// \brief Turn on a flash light specified by the light name
    /// If more than one link have lights with the identical name,
    /// the first appearing light in the list will be updated.
    /// \param[in] _light_name The name of flash light
    public: virtual bool TurnOn(const std::string &_light_name) final;

    /// \brief Turn on a flash light specified by the name and its link
    /// \param[in] _light_name The name of flash light
    /// \param[in] _link_name The name of the link holding the light
    public: virtual bool TurnOn(
      const std::string &_light_name, const std::string &_link_name) final;

    /// \brief Turn on all flash lights
    public: virtual bool TurnOnAll() final;

    /// \brief Turn off a flash light specified by the name
    /// If more than one link have lights with the identical name,
    /// the first appearing light in the list will be updated.
    /// \param[in] _light_name The name of flash light
    public: virtual bool TurnOff(const std::string &_light_name) final;

    /// \brief Turn off a flash light specified by the name
    /// \param[in] _light_name The name of flash light
    /// \param[in] _link_name The name of the link holding the light
    public: virtual bool TurnOff(
      const std::string &_light_name, const std::string &_link_name) final;

    /// \brief Turn off all flash lights
    public: virtual bool TurnOffAll() final;

    /// \brief Change the duration
    /// \param[in] _light_name The name of flash light
    /// \param[in] _link_name The name of the link holding the light
    /// \param[in] _duration The new duration time to set
    public: virtual bool ChangeDuration(
      const std::string &_light_name, const std::string &_link_name,
      const double &_duration) final;

    /// \brief Change the interval
    /// \param[in] _light_name The name of flash light
    /// \param[in] _link_name The name of the link holding the light
    /// \param[in] _interval The new interval time to set
    public: virtual bool ChangeInterval(
      const std::string &_light_name, const std::string &_link_name,
      const double &_interval) final;

    /// \brief Find a unit of settings by names
    //  This is internally used to access an individual unit of light settings
    private: virtual std::shared_ptr<FlashLightSettings> FindSettings(
      const std::string &_light_name, const std::string &_link_name) final;

    /// \brief list of light settings to control
    private: std::vector<std::shared_ptr<FlashLightSettings>> list_flash_light;

    /// \brief pointer to the update even connection
    private: event::ConnectionPtr updateConnection;
  };

  /// \brief Internal data class to hold individual flash light settings
  class FlashLightSettings
  {
    /// \brief The name of flash light
    private: std::string name;

    /// \brief Link which holds this flash light
    private: physics::LinkPtr link;

    /// \brief The time at which the current phase started
    private: common::Time start_time;

    /// \brief The current switch state (the light itself on or off)
    private: bool f_switch_on;

    /// \brief The current flasshing state (flashing or not)
    private: bool f_flashing;

    /// \brief The duration time to flash (in seconds)
    private: double duration;

    /// \brief The interval time between flashing (in seconds)
    //  When it is zero, the light is constant.
    private: double interval;

    /// \brief The length of the ray (in meters)
    private: double range;

    /// \brief The pointer to node for communication
    private: static transport::NodePtr node;

    /// \brief The pointer to publisher to send a command to a light
    private: static transport::PublisherPtr pubLight;

    /// \brief Flash the light
    /// This function is internally used to update the light in the environment
    private: void Flash();

    /// \brief Dim the light
    /// This function is internally used to update the light in the environment
    private: void Dim();

    /// \brief Constructor
    public: FlashLightSettings(
      const physics::ModelPtr &_model,
      const sdf::ElementPtr &_sdfFlashLight,
      const common::Time &_current_time);

    /// \brief Update the light based on the given time
    public: void UpdateLightInEnv(const common::Time &_current_time);

    /// \brief Getter of name
    public: const std::string Name() const;

    /// \brief Getter of link
    public: const physics::LinkPtr Link() const;

    /// \brief Switch on
    public: void SwitchOn();

    /// \brief Switch off
    public: void SwitchOff();

    /// \brief Set the duration time
    public: void SetDuration(const double &_duration);

    /// \brief Set the interval time
    public: void SetInterval(const double &_interval);
  };
}
#endif
