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
  class GAZEBO_VISIBLE FlashLightPlugin : public ModelPlugin
  {
    // friend declaration
    // This allows dataPtr of this class to create and hold objects of the
    // class, FlashLightSetting, nested in this class.
    public: friend class FlashLightPluginPrivate;

    // forward declaration
    protected: class FlashLightSettingPrivate;

    /// \brief Internal data class to hold individual flash light settings.
    /// A setting for each flash light is separately stored in a
    /// FlashLightSetting class, which takes care of dynamic specifications such
    /// as duration and interval.
    protected: class FlashLightSetting
    {
      /// \brief Constructor.
      public: FlashLightSetting();

      /// \brief Destructor.
      public: virtual ~FlashLightSetting();

      /// \brief Initialize the specific parts of FlashLightSetting.
      /// \param[in] _sdfFlashLight SDF data for flashlight settings.
      /// \param[in] _model The Model pointer holding the light to control.
      /// \param[in] _currentTime The current time point.
      public: virtual void InitFlashLightSetting(
        const sdf::ElementPtr &_sdfFlashLight,
        const physics::ModelPtr &_model,
        const common::Time &_currentTime) final;

      /// \brief Set the publisher and send an initial light command.
      /// \param[in] _pubLight The publisher to send a message
      public: virtual void InitPubLight(
        const transport::PublisherPtr &_pubLight) final;

      /// \brief Update the light based on the given time.
      /// \param[in] _currentTime The current point of time to update the
      ///                         lights.
      public:
        virtual void UpdateLightInEnv(const common::Time &_currentTime) final;

      /// \brief Getter of name.
      /// \return The name of the light element.
      public: virtual const std::string Name() const final;

      /// \brief Getter of link.
      /// \return A pointer to the link element.
      public: virtual const physics::LinkPtr Link() const final;

      /// \brief Switch on (enable the flashlight).
      public: virtual void SwitchOn() final;

      /// \brief Switch off (disable the flashlight).
      public: virtual void SwitchOff() final;

      /// \brief Set the duration time.
      /// \param[in] _duration New duration time to set.
      public: virtual void SetDuration(const double &_duration) final;

      /// \brief Set the interval time.
      /// \param[in] _interval New interval time to set.
      public: virtual void SetInterval(const double &_interval) final;

      /// \brief Flash the light
      /// This function is internally used to update the light in the
      /// environment.
      protected: virtual void Flash();

      /// \brief Dim the light
      /// This function is internally used to update the light in the
      /// environment.
      protected: virtual void Dim();

      /// \brief Pointer to private data
      protected: std::unique_ptr<FlashLightSettingPrivate> dataPtr;
    };

    /// \brief Constructor.
    public: FlashLightPlugin();

    /// \brief Destructor.
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

    /// \brief Create an object of setting.
    ///
    /// NOTE: This function is internally called in Load().
    /// If a child class of FlashLightPlugin has also an inherited class of
    /// FlashLightSetting, this function must be overridden so that dataPtr
    /// deals with objects of the appropriate setting class.
    ///
    /// \param[in] _sdfFlashLight SDF data for flashlight settings.
    /// \param[in] _currentTime The current time point.
    /// \return A pointer to the newly created setting object.
    protected: virtual std::shared_ptr<FlashLightSetting> CreateSetting();

    /// \brief Initialize the additional part of an object of setting.
    ///
    /// NOTE: This function is internally called in Load().
    /// If a child class of FlashLightPlugin has also an inherited class of
    /// FlashLightSetting, this function must be overridden so that the object
    /// can be initialized with necessary data.
    protected:
      virtual void InitAdditionalSetting(std::shared_ptr<FlashLightSetting> &_setting);

    /// \brief Pointer to private data
    private: std::unique_ptr<FlashLightPluginPrivate> dataPtr;
  };
}
#endif
