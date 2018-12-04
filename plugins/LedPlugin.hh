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

#ifndef GAZEBO_PLUGINS_LEDPLUGIN_HH_
#define GAZEBO_PLUGINS_LEDPLUGIN_HH_

#include <memory>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"

#include "FlashLightPlugin.hh"

namespace gazebo
{
  // forward declaration
  class LedSettingPrivate;

  /// \brief Internal data class to hold individual LED light settings.
  class GAZEBO_VISIBLE LedSetting: public FlashLightSetting
  {
    /// \brief Constructor.
    public: LedSetting(
      const sdf::ElementPtr &_sdf,
      const physics::ModelPtr &_model,
      const common::Time &_currentTime);

    /// \brief Destructor.
    public: virtual ~LedSetting();

    /// \brief Set the publisher and send an initial visual command.
    /// \param[in] _pubVisual The publisher to send a message
    public: virtual void InitPubVisual(
      const transport::PublisherPtr &_pubVisual) final;

    // Documentation inherited.
    protected: virtual void Flash();

    // Documentation inherited.
    protected: virtual void Dim();

    /// \brief Pointer to private data
    private: std::unique_ptr<LedSettingPrivate> dataPtr;
  };

  // forward declaration
  class LedPluginPrivate;

  /// \brief A plugin that blinks light and visual elements in a model.
  /// In addition to the features of the base plugin, FlashLightPlugin, this
  /// plugin accesses a <visual> element to make it blink.
  ///
  /// The <light> and <visual> element to control must have the identical name
  /// and specified by <light_id>, including the name of the link, which they
  /// are attached on, and their name separated by a slash "/".
  ///
  /// Other parameters are inherited from the base plugin, FlashLightPlugin.
  ///
  /// Example:
  /// \verbatim
  /// <model name='light_model'>
  ///   ...
  ///   <link name='light_source'>
  ///     ...
  ///     <light name='lamp' type='point'>
  ///       ...
  ///     </light>
  ///     <visual name='lamp'>
  ///       ...
  ///     </visual>
  ///   </link>
  ///   ...
  ///   <plugin name='light_control' filename='libLedPlugin.so'>
  ///     <light>
  ///       <id>light_source/lamp</id>
  ///       <block>
  ///         <duration>1</duration>
  ///         <interval>1</interval>
  ///         <color>1 0.5 0.5</color>
  ///       </block>
  ///       <enable>true</enable>
  ///     </light>
  ///   </plugin>
  /// </model>
  /// \endverbatim
  ///
  class GAZEBO_VISIBLE LedPlugin : public FlashLightPlugin
  {
    /// \brief Constructor.
    public: LedPlugin();

    /// \brief Destructor.
    public: virtual ~LedPlugin();

    // Documentation inherited.
    protected: virtual std::shared_ptr<FlashLightSetting> CreateSetting(
      const sdf::ElementPtr &_sdf,
      const physics::ModelPtr &_model,
      const common::Time &_currentTime);

    // Documentation inherited.
    protected: virtual void InitSettingBySpecificData(
        std::shared_ptr<FlashLightSetting> &_setting);

    /// \brief Pointer to private data
    private: std::unique_ptr<LedPluginPrivate> dataPtr;
  };
}
#endif
