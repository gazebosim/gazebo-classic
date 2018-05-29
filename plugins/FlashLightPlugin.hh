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
#include <memory>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"

namespace gazebo
{
  // forward declaration
  class FlashLightPluginPrivate;

  /// \brief A plugin that turns on/off a light component in the model.
  // This plugin accesses <light> elements in the model specified
  // by <flash_light> as a parameter.
  // The light is specified by <light_id>, including link and light names
  // separated by a slash "/".
  // <start> is optional. It indicates whehter it starts flashing from the
  // beginning. If <start> is directly under the <plugin>, it affects all
  // the lights. Otherwise, it individually affects the corresponding <light>
  // element.
  //
  // <start>true</start>
  // <flash_light>
  //  <light_id>link1/light_source</light_id>
  //  <duration>0.1</duration>
  //  <interval>0.4</interval>
  //  <start>true</start>
  // </flash_light>
  // <flash_light>
  //  <light_id>link1/light_source2</light_id>
  //  <duration>0.8</duration>
  //  <interval>0.2</interval>
  //  <start>false</start>
  // </flash_light>
  // ...
  //
  // More than one <flash_light> can exist.
  //
  // Settings for each flash light is separately stored in a
  // FlashLightSettings class, which takes care of dynamic specifications
  // such as duration and interval.
  //
  // NOTE: This base class provides basic functions to turn the lights on/off.
  // Users can create their own flash light plugin by inheriting this base
  // model.
  //
  class GAZEBO_VISIBLE FlashLightPlugin : public ModelPlugin
  {
    // Constructor
    public: FlashLightPlugin();
    // Destructor
    public: ~FlashLightPlugin();

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

    /// \brief Pointer to private data
    private: std::unique_ptr<FlashLightPluginPrivate> dataPtr;
  };
}
#endif
