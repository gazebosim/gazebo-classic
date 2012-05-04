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

#include "physics/physics.h"
#include "transport/transport.h"
#include "plugins/GripperPlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(GripperPlugin)

/////////////////////////////////////////////////
GripperPlugin::GripperPlugin()
{
}

/////////////////////////////////////////////////
void GripperPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->model = _model;

  sdf::ElementPtr jointElem = _sdf->GetElement("joint");

  while (jointElem)
  {
    std::cout << "Joint[" << jointElem->GetValueString() << "]\n";
    jointElem = jointElem->GetNextElement("joint");
  }
}

/////////////////////////////////////////////////
void GripperPlugin::Init()
{
}

/////////////////////////////////////////////////
void GripperPlugin::OnUpdate()
{
}
