/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#ifndef _RENDERINGIFACE_HH_
#define _RENDERINGIFACE_HH_

#include <string>
#include "gazebo/msgs/poses_stamped.pb.h"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{

    /// \brief load rendering engine.
    GZ_RENDERING_VISIBLE
    bool load();

    /// \brief init rendering engine.
    GZ_RENDERING_VISIBLE
    bool init();

    /// \brief teardown rendering engine.
    GZ_RENDERING_VISIBLE
    bool fini();

    /// \brief get pointer to rendering::Scene by name.
    /// \param[in] _name Name of the scene to retrieve.
    GZ_RENDERING_VISIBLE
    rendering::ScenePtr get_scene(const std::string &_name = "");

    /// \brief Update Poses via direct API call instead of transport.
    /// A pointer to this function is passed to physics when initializing
    /// a Gazebo Server.
    /// \param[in] _name Name of the scene concerned.
    /// \param[in] _msg message to be passed.
    GZ_RENDERING_VISIBLE
    void update_scene_poses(const std::string &_name,
                            const msgs::PosesStamped &_msg);

    /// \brief Set whether to enable lockstepping for rendering and physics.
    /// If enabled, the poses of objects in rendering will be updated via
    /// direct API call instead of transport.
    /// \param[in] _enable True to enable lockstepping, false to disable
    /// \sa update_scene_poses
    GZ_RENDERING_VISIBLE
    void set_lockstep_enabled(bool _enable);

    /// \brief Get whether or not lockstepping is enabled for rendering and
    /// physics. If enabled, the poses of objects in rendering is updated via
    /// direct API call instead of transport.
    /// \return True if lockstepping is enabled, false if disabled
    /// \sa lockstep_enabled
    GZ_RENDERING_VISIBLE
    bool lockstep_enabled();

    /// \brief wait until a render request occurs
    /// \param[in] _name Name of the scene to retrieve
    /// \param[in] _timeoutsec timeout expressed in seconds
    /// \return true if a render request occured, false in case
    ///          we waited until the timeout
    GZ_RENDERING_VISIBLE
    bool wait_for_render_request(const std::string &_name,
                                 const double _timeoutsec);

    /// \brief create rendering::Scene by name.
    /// \param[in] _name Name of the scene to create.
    /// \param[in] _enableVisualizations True enables visualization
    /// elements such as laser lines.
    GZ_RENDERING_VISIBLE
    rendering::ScenePtr create_scene(const std::string &_name,
                                     bool _enableVisualizations,
                                     bool _isServer = false);

    /// \brief remove a rendering::Scene by name
    /// \param[in] _name The name of the scene to remove.
    GZ_RENDERING_VISIBLE
    void remove_scene(const std::string &_name);

    /// \}
  }
}
#endif
