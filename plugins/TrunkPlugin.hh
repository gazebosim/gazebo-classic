/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#ifndef GAZEBO_PLUGINS_TRUNKPLUGIN_HH_
#define GAZEBO_PLUGINS_TRUNKPLUGIN_HH_

#include <memory>

#include "gazebo/common/Plugin.hh"

namespace gazebo
{
  // Forward declare private data class
  class TrunkPluginPrivate;

  class GAZEBO_VISIBLE TrunkPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: TrunkPlugin();

    /// \brief Destructor.
    public: ~TrunkPlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback for World Update events.
    private: virtual void OnUpdate();

    // Private data pointer.
    private: std::unique_ptr<TrunkPluginPrivate> dataPtr;
  };
}
#endif
