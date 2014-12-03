/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
    /// \param[in] _name Name of the scene to retreive.
    GZ_RENDERING_VISIBLE
    rendering::ScenePtr get_scene(const std::string &_name = "");

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
