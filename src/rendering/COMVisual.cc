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

#include "rendering/DynamicLines.hh"
#include "rendering/ogre.h"
#include "rendering/Scene.hh"
#include "rendering/COMVisual.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
COMVisual::COMVisual(const std::string &_name, VisualPtr _vis)
  : Visual(_name, _vis, false)
{
}

/////////////////////////////////////////////////
COMVisual::~COMVisual()
{
  delete this->crossLines;
  this->crossLines = NULL;

}

/////////////////////////////////////////////////
void COMVisual::Load()
{
  Visual::Load();

  this->crossLines = this->CreateDynamicLine(rendering::RENDERING_LINE_LIST);
  this->crossLines->setMaterial("Gazebo/Green");

  this->crossLines->AddPoint(math::Vector3(0, 0, -0.04));
  this->crossLines->AddPoint(math::Vector3(0, 0, 0.04));

  this->crossLines->AddPoint(math::Vector3(0, -0.04, 0));
  this->crossLines->AddPoint(math::Vector3(0, 0.04, 0));

  this->crossLines->AddPoint(math::Vector3(-0.04, 0, 0));
  this->crossLines->AddPoint(math::Vector3(0.04, 0, 0));

  Ogre::MovableObject *boxObj =
    (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
          this->GetName()+"__BOX__", "unit_box"));
  boxObj->setVisibilityFlags(GZ_VISIBILITY_GUI);
  ((Ogre::Entity*)boxObj)->setMaterialName("__GAZEBO_TRANS_PURPLE_MATERIAL__");

  this->boxNode =
    this->sceneNode->createChildSceneNode(this->GetName() + "_BOX");

  this->boxNode->attachObject(boxObj);
  this->boxNode->setScale(0.02, 0.02, 0.02);

  this->SetVisibilityFlags(GZ_VISIBILITY_GUI);
}
