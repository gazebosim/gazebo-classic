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
#include "plugins/FlashLightPlugin.hh"

namespace gazebo
{
  /// \brief A plugin that blinks a light component in the model.
  /// This plugin accesses <light> elements in the model specified by
  /// <flash_light> as a parameter. More than one <flash_light> can exist.
  ///
  /// NOTE: This base class provides basic functions to control the lights.
  /// Users can create their own flash light plugin by inheriting this base
  /// model plugin.
  ///
  /// A light is specified by <light_id>, including link and light names
  /// separated by a slash "/".
  ///
  /// <enable> is optional. The default value is true.
  /// When it is set to true, a light starts flashing from the beginning.
  /// If <enable> is directly under the <plugin>, it affects all the lights.
  /// If it is under <flash_light>, it individually affects the corresponding
  /// light.
  /// When it is set to false, the light is off, and it is  necessary to call
  /// TurnOn() or TurnOnAll() in an inherited class to enable the light.
  /// A locally placed <enable> has a higher priority than the global one so
  /// specific lights can be turned on while the others are all off, and vice
  /// versa.
  ///
  /// <duration> and <interval> specify the time to flash and dim in seconds,
  /// respectively. That is, the phase is determined as the sum of them:
  /// duration + interval.
  ///
  /// Example:
  /// \verbatim
  /// <enable>true</enable>
  ///
  /// <flash_light>
  ///   <light_id>link1/light_source</light_id>
  ///   <duration>0.1</duration>
  ///   <interval>0.4</interval>
  ///   <enable>true</enable>
  /// </flash_light>
  ///
  /// <flash_light>
  ///   <light_id>link1/light_source2</light_id>
  ///   <duration>0.8</duration>
  ///   <interval>0.2</interval>
  ///   <enable>false</enable>
  /// </flash_light>
  ///
  /// ...
  /// \endverbatim
  ///
  class GAZEBO_VISIBLE LEDPlugin : public FlashLightPlugin
  {
    /// \brief Internal data class to hold individual flash light settings.
    /// A setting for each flash light is separately stored in a
    /// LEDSetting class, which takes care of dynamic specifications such
    /// as duration and interval.
    protected: class LEDSetting: public FlashLightSetting
    {
      /// \brief Constructor.
      /// \param[in] _model The Model pointer holding the light to control.
      /// \param[in] _pubLight The publisher for the light element.
      /// \param[in] _sdfFlashLight SDF data for flashlight settings.
      /// \param[in] _currentTime The current time point.
      /// \param[in] _pubVisual The publisher for the visual element.
      public: LEDSetting(
        const physics::ModelPtr &_model,
        const transport::PublisherPtr &_pubLight,
        const sdf::ElementPtr &_sdfFlashLight,
        const common::Time &_currentTime,
        const transport::PublisherPtr &_pubVisual);

      /// \brief Destructor.
      public: virtual ~LEDSetting();

      /// \brief Flash the light
      /// This function is internally used to update the light in the
      /// environment.
      protected: virtual void Flash();

      /// \brief Dim the light
      /// This function is internally used to update the light in the
      /// environment.
      protected: virtual void Dim();
    };

    /// \brief Constructor.
    public: LEDPlugin();

    /// \brief Destructor.
    public: virtual ~LEDPlugin();

    /// \brief Create an object of setting.
    ///
    /// NOTE: This function is internally called in Load().
    /// If a child class of FlashLightPlugin has also an inherited class of
    /// FlashLightSetting, this function must be overridden so that dataPtr
    /// deals with objects of the appropriate setting class.
    ///
    /// \param[in] _model The Model pointer holding the light to control.
    /// \param[in] _pubLight The publisher to send a message
    /// \param[in] _sdfFlashLight SDF data for flashlight settings.
    /// \param[in] _currentTime The current time point.
    /// \return A pointer to the newly created setting object.
    protected: virtual std::shared_ptr<FlashLightSetting> CreateSetting(
      const sdf::ElementPtr &_sdfFlashLight,
      const common::Time &_currentTime);
  };
}
#endif
