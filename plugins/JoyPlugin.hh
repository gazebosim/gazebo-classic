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
#ifndef GAZEBO_PLUGINS_JOYPLUGIN_HH_
#define GAZEBO_PLUGINS_JOYPLUGIN_HH_

#include <gazebo/common/Plugin.hh>

namespace gazebo
{
  class JoyPluginPrivate;

  /// \brief The JoyPlugin connects to a joystick or gamepad, and transmits
  /// data from the joystick over an Ignition Transport topic. The default
  /// topic name is /joy, and the message type is ignition.msgs.Joy.
  ///
  /// Another plugin or application can listen to the joystick messages, and
  /// then take actions based on the data.
  ///
  /// # Example
  ///
  /// 1. Connect a joystick to the computer.
  ///
  /// 2. Run the Gazebo demo: gazebo worlds/joy_demo.world
  ///
  /// 3. Echo the joy data to a terminal: ign topic -e -t /joy
  ///
  ///    Note: You will need
  /// [ign-tools](https://bitbucket.org/ignitionrobotics/ign-tools)
  ///
  /// # Usage
  ///
  /// The plugin is loaded via a world plugin. In SDF this looks like:
  ///
  /// \code
  /// <plugin name="joy" filename="libJoyPlugin.so">
  ///    <sticky_buttons>false</sticky_buttons>
  ///    <dead_zone>0.05</dead_zone>
  ///    <rate>60</rate>
  ///    <accumulation_rate>1000</accumulation_rate>
  ///  </plugin>
  /// \endcode
  ///
  ///
  /// See worlds/joy_demo.world for an example.
  ///
  /// # Configuration
  ///
  /// Options that can be specified in the plugin include:
  ///
  /// 1. <dev>(string): Name of the joystick device. The default value is
  /// /dev/input/js0.
  ///
  /// 2. <topic>(string): Name of the topic on which to publish joy
  /// messages. The default is /joy.
  ///
  /// 3. <sticky_buttons>(true/false): When true the buttons values change
  /// state only on a transition from 0 to 1. This makes the button act like
  /// toggle buttons. False is the default
  ///
  /// 4. <dead_zone>(float between 0 and 0.9): A larger number increases the
  /// distance the joystick has to move before registering a value. The
  /// default is 0.05;
  ///
  /// 5. <rate>(float): The rate at which joystick messages are published,
  /// in Hz. The default value is 1Hz.
  ///
  /// 6. <accumulation_rate>(float): The rate at which data is collected
  /// from the joystick device. The default value is 1000Hz.
  ///
  /// # Troubleshooting
  ///
  /// 1. Playstation joysticks may require a press of the center "Ps" button
  /// before they work.
  ///
  class GAZEBO_VISIBLE JoyPlugin : public WorldPlugin
  {
    /// \brief Constructor.
    public: JoyPlugin();

    /// \brief Destructor.
    public: virtual ~JoyPlugin();

    // Documentation Inherited.
    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override;

    /// \brief Private data pointer.
    private: JoyPluginPrivate *dataPtr = nullptr;
  };
}
#endif
