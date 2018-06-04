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

#include <memory>
#include <string>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"

namespace gazebo
{
  // forward declaration
  class FlashLightPluginPrivate;

  /// \brief A plugin that blinks a light component in the model.
  /// This plugin accesses <light> elements in the model specified
  /// by <flash_light> as a parameter.
  ///
  /// NOTE: This base class provides basic functions to control the lights.
  /// Users can create their own flash light plugin by inheriting this base
  /// model plugin.
  ///
  /// A light is specified by <light_id>, including link and light names
  /// separated by a slash "/".
  ///
  /// <enable> is optional. It indicates whether a light starts flashing from
  /// the beginning. If <enable> is directly under the <plugin>, it affects all
  /// the lights. Otherwise, it individually affects the corresponding <light>
  /// element. If <enable> is false, the light is off, and it is  necessary to
  /// call TurnOn() or TurnOnAll() in an inherited class to enable the light.
  /// If <enable> is not given, the light is enabled in default.
  ///
  /// <duration> and <interval> represent the time to flash and dim in seconds,
  /// respectively. That is, the phase is determined as the sum of them:
  /// duration + interval.
  ///
  /// Example:
  /// <enable>true</enable>
  /// <flash_light>
  ///  <light_id>link1/light_source</light_id>
  ///  <duration>0.1</duration>
  ///  <interval>0.4</interval>
  ///  <enable>true</enable>
  /// </flash_light>
  /// <flash_light>
  ///  <light_id>link1/light_source2</light_id>
  ///  <duration>0.8</duration>
  ///  <interval>0.2</interval>
  ///  <enable>false</enable>
  /// </flash_light>
  /// ...
  ///
  /// More than one <flash_light> can exist.
  ///
  class GAZEBO_VISIBLE FlashLightPlugin : public ModelPlugin
  {
    // Constructor
    public: FlashLightPlugin();
    // Destructor
    public: virtual ~FlashLightPlugin();

    // Documentation inherited.
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;

    /// \brief Called by the world update start event
    protected: virtual void OnUpdate();

    /// \brief Turn on a flash light specified by the light name
    /// If more than one link have lights with the identical name,
    /// the first appearing light in the list will be updated.
    /// \param[in] _lightName The name of flash light
    /// \return True if the specified light is found.
    protected: virtual bool TurnOn(const std::string &_lightName) final;

    /// \brief Turn on a flash light specified by the name and its link
    /// \param[in] _lightName The name of flash light
    /// \param[in] _linkName The name of the link holding the light
    /// \return True if the specified light is found.
    protected: virtual bool TurnOn(
      const std::string &_lightName, const std::string &_linkName) final;

    /// \brief Turn on all flash lights
    /// \return True if there is one or more lights to turn on.
    protected: virtual bool TurnOnAll() final;

    /// \brief Turn off a flash light specified by the name
    /// If more than one link have lights with the identical name,
    /// the first appearing light in the list will be updated.
    /// \param[in] _lightName The name of flash light
    /// \return True if the specified light is found.
    protected: virtual bool TurnOff(const std::string &_lightName) final;

    /// \brief Turn off a flash light specified by the name
    /// \param[in] _lightName The name of flash light
    /// \param[in] _linkName The name of the link holding the light
    /// \return True if the specified light is found.
    protected: virtual bool TurnOff(
      const std::string &_lightName, const std::string &_linkName) final;

    /// \brief Turn off all flash lights
    /// \return True if there is one or more lights to turn off.
    protected: virtual bool TurnOffAll() final;

    /// \brief Change the duration
    /// \param[in] _lightName The name of flash light
    /// \param[in] _linkName The name of the link holding the light
    /// \param[in] _duration The new duration time to set
    /// \return True if the specified light is found.
    protected: virtual bool ChangeDuration(
      const std::string &_lightName, const std::string &_linkName,
      const double &_duration) final;

    /// \brief Change the interval
    /// \param[in] _lightName The name of flash light
    /// \param[in] _linkName The name of the link holding the light
    /// \param[in] _interval The new interval time to set
    /// \return True if the specified light is found.
    protected: virtual bool ChangeInterval(
      const std::string &_lightName, const std::string &_linkName,
      const double &_interval) final;

    /// \brief Pointer to private data
    private: std::unique_ptr<FlashLightPluginPrivate> dataPtr;
  };
}
#endif
