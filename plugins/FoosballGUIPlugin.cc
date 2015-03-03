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

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include "FoosballGUIPlugin.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(FoosballGUIPlugin)

/////////////////////////////////////////////////
FoosballGUIPlugin::FoosballGUIPlugin()
  : GUIPlugin()
{
}

/////////////////////////////////////////////////
FoosballGUIPlugin::~FoosballGUIPlugin()
{
}

/////////////////////////////////////////////////
void FoosballGUIPlugin::Load(sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_sdf, "FoosballGUIPlugin _sdf pointer is NULL");
  this->sdf = _sdf;
}
