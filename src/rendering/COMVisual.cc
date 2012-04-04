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
/* Desc: Center of Mass Visualization Class
 * Author: Nate Koenig
 */

#include "common/MeshManager.hh"

#include "rendering/ogre.h"
#include "rendering/Scene.hh"
#include "rendering/COMVisual.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
COMVisual::COMVisual(const std::string &_name, VisualPtr _vis)
  : Visual(_name, _vis)
{
}

/////////////////////////////////////////////////
COMVisual::~COMVisual()
{
}

/////////////////////////////////////////////////
void COMVisual::Load()
{
  Visual::Load();

  this->AttachMesh("unit_box");
  this->SetScale(math::Vector3(0.02, 0.02, 0.02));
  this->SetMaterial("Gazebo/RedTransparent", false);

  this->SetVisibilityFlags(GZ_VISIBILITY_GUI);
}
