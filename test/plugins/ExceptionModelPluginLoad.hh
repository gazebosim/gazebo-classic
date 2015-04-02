/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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
#ifndef _GAZEBO_EXCEPTION_MODEL_PLUGIN_LOAD_HH_
#define _GAZEBO_EXCEPTION_MODEL_PLUGIN_LOAD_HH_

#include <string>

#include "gazebo/common/Plugin.hh"
#include "gazebo/gazebo.hh"

namespace gazebo
{
  class ExceptionModelPluginLoad : public ModelPlugin
  {
    /// \brief Constructor
    public: ExceptionModelPluginLoad();

    /// \brief Destructor
    public: virtual ~ExceptionModelPluginLoad();

    // Documentation inherited
    public: virtual void Load(physics::ModelPtr _mode, sdf::ElementPtr _sdf);
  };
}
#endif
