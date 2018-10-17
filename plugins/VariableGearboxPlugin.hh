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
#ifndef GAZEBO_VARIABLE_GEARBOX_PLUGIN_HH
#define GAZEBO_VARIABLE_GEARBOX_PLUGIN_HH

#include <memory>
#include <sdf/Element.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/util/system.hh>

namespace gazebo
{
  class VariableGearboxPluginPrivate;

  class GAZEBO_VISIBLE VariableGearboxPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: VariableGearboxPlugin();

    /// \brief Destructor
    public: virtual ~VariableGearboxPlugin();

    /// \brief Load the controller
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the gear ratios based on current joint position.
    /// \param[in] _info Update information provided by the server.
    private: void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Private data pointer.
    private: std::unique_ptr<VariableGearboxPluginPrivate> dataPtr;
  };
}
#endif
