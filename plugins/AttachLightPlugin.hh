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

#ifndef GAZEBO_PLUGINS_ATTACHLIGHTPLUGIN_HH_
#define GAZEBO_PLUGINS_ATTACHLIGHTPLUGIN_HH_

#include <memory>

#include "gazebo/common/Plugin.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  // forward declarations
  class AttachLightPluginPrivate;

  /// \brief A model plugin that enables multiple lights in the world to be
  /// attached to links within the model.
  ///
  /// The plugin has the following SDF description:
  /// <link>            SDF element. More than one <link> can be defined.
  ///   <link_name>     Name of the link in the model.
  ///   <light>         SDF element. More than one <light> can be attached.
  ///     <pose>        Offset pose of the light in the link frame.
  ///     <light_name>  Name of the light in the world.
  class GAZEBO_VISIBLE AttachLightPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: AttachLightPlugin();

    // Documentation inherited
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Main loop to update the pose of lights
    private: void OnUpdate();

    /// \brief Request callback for handling deletion events.
    /// \param[in] _msg The message data.
    public: void OnRequest(ConstRequestPtr &_msg);

    /// \brief Pointer to private data
    private: std::unique_ptr<AttachLightPluginPrivate> dataPtr;
  };
}
#endif
