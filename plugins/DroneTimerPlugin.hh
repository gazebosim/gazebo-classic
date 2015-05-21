/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef _GAZEBO_DRONE_TIMER_PLUGIN_HH_
#define _GAZEBO_DRONE_TIMER_PLUGIN_HH_

#include <string>
#include <vector>

#include "gazebo/common/Event.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  namespace rendering
  {
    class MovableText;
  }

  /// \brief A plugin that simulates lift and drag.
  class DroneTimerPlugin : public VisualPlugin
  {
    /// \brief Constructor.
    public: DroneTimerPlugin();

    /// \brief Destructor.
    public: ~DroneTimerPlugin();

    // Documentation Inherited.
    public: virtual void Load(rendering::VisualPtr _vis, sdf::ElementPtr _sdf);

    /// \brief Reset the plugin.
    public: virtual void Reset();

    private: void Update();

    private: rendering::VisualPtr vis;

    private: rendering::MovableText *text;

    private: event::ConnectionPtr updateConnection;
  };
}
#endif
