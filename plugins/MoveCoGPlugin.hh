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
#ifndef GAZEBO_PLUGINS_MOVECOGPLUGIN_HH_
#define GAZEBO_PLUGINS_MOVECOGPLUGIN_HH_

#include <gazebo/common/Plugin.hh>

namespace gazebo
{
  /// \brief Hard-coded test of moving a link's CoG.
  class GAZEBO_VISIBLE MoveCoGPlugin : public ModelPlugin
  {
    /// \brief Send a message to modify the CoG of link "link2".
    public: virtual void Load(physics::ModelPtr _model,
                sdf::ElementPtr /*_sdf*/);

    /// \brief Store the plugin's model and set the update callback.
    public: virtual void Init();

    /// \brief Move the CoG of link "link" after 10s.
    private: void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Connection to update callback so it can be disconnected.
    private: event::ConnectionPtr updateConnection;

    /// \brief The model.
    private: physics::ModelPtr model;

  };
}
#endif
