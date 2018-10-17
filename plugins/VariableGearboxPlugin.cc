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

#include <boost/bind.hpp>
#include <functional>
#include <gazebo/common/Events.hh>
#include "VariableGearboxPlugin.hh"

namespace gazebo
{
class VariableGearboxPluginPrivate
{
  /// \brief Parent model pointer.
  public: physics::ModelPtr model;

  /// \brief World update connection.
  public: event::ConnectionPtr updateConnection;
};

/////////////////////////////////////////////////
VariableGearboxPlugin::VariableGearboxPlugin()
  : dataPtr(new VariableGearboxPluginPrivate)
{
}

/////////////////////////////////////////////////
VariableGearboxPlugin::~VariableGearboxPlugin()
{
}

/////////////////////////////////////////////////
void VariableGearboxPlugin::Load(physics::ModelPtr _parent,
                                 sdf::ElementPtr /*_sdf*/)
{
  this->dataPtr->model = _parent;

  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&VariableGearboxPlugin::OnUpdate, this, _1));
}

/////////////////////////////////////////////////
void VariableGearboxPlugin::OnUpdate(const common::UpdateInfo & /*_info*/)
{
}

GZ_REGISTER_MODEL_PLUGIN(VariableGearboxPlugin)
}
