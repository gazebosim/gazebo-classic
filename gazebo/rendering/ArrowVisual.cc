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
/* Desc: Arrow Visualization Class
 * Author: Nate Koenig
 */

#include "gazebo/common/MeshManager.hh"

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/ArrowVisual.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
ArrowVisual::ArrowVisual(const std::string &_name, VisualPtr _vis)
  : Visual(_name, _vis, false)
{
  this->headNode = NULL;
  this->shaftNode = NULL;
  this->rotationNode = NULL;
}

/////////////////////////////////////////////////
ArrowVisual::~ArrowVisual()
{
}

/////////////////////////////////////////////////
void ArrowVisual::Load()
{
  Visual::Load();

  // Make sure the meshes are in Ogre
  this->InsertMesh("axis_shaft");
  this->InsertMesh("axis_head");

  Ogre::MovableObject *shaftObj =
    (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
          this->GetName()+"__SHAFT__", "axis_shaft"));

  Ogre::MovableObject *headObj =
    (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
          this->GetName()+"__HEAD__", "axis_head"));

  this->shaftNode =
    this->sceneNode->createChildSceneNode(this->GetName() + "_SHAFT");
  this->shaftNode->attachObject(shaftObj);
  this->shaftNode->setPosition(0, 0, 0.1);

  this->headNode =
    this->sceneNode->createChildSceneNode(this->GetName() + "_HEAD");
  this->headNode->attachObject(headObj);
  this->headNode->setPosition(0, 0, 0.24);

  this->SetVisibilityFlags(GZ_VISIBILITY_GUI);
}

/////////////////////////////////////////////////
void ArrowVisual::ShowRotation()
{
  common::MeshManager::Instance()->CreateTube("rotation_tube",
      0.035, 0.04, 0.01, 1, 32);
  this->InsertMesh("rotation_tube");

  Ogre::MovableObject *rotationObj =
    (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
          this->GetName()+"__ROTATION__", "rotation_tube"));
  rotationObj->setVisibilityFlags(GZ_VISIBILITY_GUI);
  ((Ogre::Entity*)rotationObj)->setMaterialName(this->GetMaterialName());

  this->rotationNode =
    this->sceneNode->createChildSceneNode(this->GetName() + "_ROTATION");
  this->rotationNode->attachObject(rotationObj);
  this->rotationNode->setPosition(0, 0, 0.24);
  this->rotationNode->setVisible(this->GetVisible());
}
