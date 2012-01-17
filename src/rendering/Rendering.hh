/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#ifndef RENDERING_HH
#define RENDERING_HH

#include "RenderTypes.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{
    bool load();
    bool init();
    bool fini();

    rendering::ScenePtr get_scene(const std::string &_name);
    rendering::ScenePtr create_scene(const std::string &name,
                                     bool _enableVisualizations);
    void remove_scene(const std::string &name);
    /// \}
  }
}
#endif

