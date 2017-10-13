/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include <ignition/transport/Node.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>

namespace gazebo
{
  class GAZEBO_VISIBLE JoyPlugin : public WorldPlugin
  {
    /// \brief Constructor.
    public: JoyPlugin();

    /// \brief Destructor.
    public: virtual ~JoyPlugin();

    // Documentation Inherited.
    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Update the joystick data.
    private: void Update();

    /// \brief Joystick device file descriptor
    private: int joyFd;

    /// \brief Ingnition communication node pointer.
    private: ignition::transport::Node node;

    /// \brief Publisher pointer used to publish the messages.
    private: ignition::transport::Node::Publisher pub;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: ignition::msgs::Joy joyMsg;
    private: ignition::msgs::Joy lastJoyMsg;
    private: ignition::msgs::Joy stickyButtonsJoyMsg;
    private: float unscaledDeadzone;
    private: float axisScale;
  };
}
#endif
