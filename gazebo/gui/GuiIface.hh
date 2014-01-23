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
#ifndef _GAZEBO_GUIIFACE_HH_
#define _GAZEBO_GUIIFACE_HH_

#include <string>
#include "gazebo/rendering/RenderingIface.hh"

namespace gazebo
{
  namespace gui
  {
    void load();

    void init();

    bool run(int _argc, char **_argv);
    void stop();

    void set_world(const std::string& _name);
    std::string get_world();

    void set_active_camera(rendering::UserCameraPtr _cam);
    rendering::UserCameraPtr get_active_camera();
    void clear_active_camera();

    unsigned int get_entity_id(const std::string &_name);
    bool has_entity_name(const std::string &_name);
  }
}
#endif
