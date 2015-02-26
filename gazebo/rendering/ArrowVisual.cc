/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include "gazebo/rendering/ArrowVisualPrivate.hh"
#include "gazebo/rendering/ArrowVisual.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
ArrowVisual::ArrowVisual(const std::string &_name, VisualPtr _vis)
  : Visual(*new ArrowVisualPrivate, _name, _vis, false)
{
  ArrowVisualPrivate *dPtr =
      reinterpret_cast<ArrowVisualPrivate *>(this->dataPtr);

  dPtr->headNode = NULL;
  dPtr->shaftNode = NULL;
  dPtr->rotationNode = NULL;
}

/////////////////////////////////////////////////
ArrowVisual::~ArrowVisual()
{
}

/////////////////////////////////////////////////
void ArrowVisual::Load()
{
  Visual::Load();

  ArrowVisualPrivate *dPtr =
      reinterpret_cast<ArrowVisualPrivate *>(this->dataPtr);

  // Make sure the meshes are in Ogre
  this->InsertMesh("axis_shaft");
  this->InsertMesh("axis_head");

  Ogre::MovableObject *shaftObj =
    (Ogre::MovableObject*)(dPtr->scene->GetManager()->createEntity(
          this->GetName()+"__SHAFT__", "axis_shaft"));

  Ogre::MovableObject *headObj =
    (Ogre::MovableObject*)(dPtr->scene->GetManager()->createEntity(
          this->GetName()+"__HEAD__", "axis_head"));

  dPtr->shaftNode =
      dPtr->sceneNode->createChildSceneNode(
      this->GetName() + "_SHAFT");
  dPtr->shaftNode->attachObject(shaftObj);
  dPtr->shaftNode->setPosition(0, 0, 0.1);

  dPtr->headNode =
      dPtr->sceneNode->createChildSceneNode(
      this->GetName() + "_HEAD");
  dPtr->headNode->attachObject(headObj);
  dPtr->headNode->setPosition(0, 0, 0.24);

  common::MeshManager::Instance()->CreateTube("rotation_tube",
      0.035, 0.04, 0.01, 1, 32);
  this->InsertMesh("rotation_tube");

  Ogre::MovableObject *rotationObj =
    (Ogre::MovableObject*)(dPtr->scene->GetManager()->createEntity(
          this->GetName()+"__ROTATION__", "rotation_tube"));

  dPtr->rotationNode =
      dPtr->sceneNode->createChildSceneNode(
      this->GetName() + "_ROTATION");
  dPtr->rotationNode->attachObject(rotationObj);
  dPtr->rotationNode->setPosition(0, 0, 0.24);
  this->ShowRotation(false);

  this->SetVisibilityFlags(GZ_VISIBILITY_GUI);
}

/////////////////////////////////////////////////
void ArrowVisual::ShowShaft(bool _show)
{
  ArrowVisualPrivate *dPtr =
      reinterpret_cast<ArrowVisualPrivate *>(this->dataPtr);

  if (_show)
  {
    dPtr->sceneNode->addChild(dPtr->shaftNode);
  }
  else
  {
    dPtr->sceneNode->removeChild(dPtr->shaftNode);
  }
}

/////////////////////////////////////////////////
void ArrowVisual::ShowHead(bool _show)
{
  ArrowVisualPrivate *dPtr =
      reinterpret_cast<ArrowVisualPrivate *>(this->dataPtr);

  if (_show)
  {
    dPtr->sceneNode->addChild(dPtr->headNode);
  }
  else
  {
    dPtr->sceneNode->removeChild(dPtr->headNode);
  }
}

/////////////////////////////////////////////////
void ArrowVisual::ShowRotation(bool _show)
{
  ArrowVisualPrivate *dPtr =
      reinterpret_cast<ArrowVisualPrivate *>(this->dataPtr);

  if (_show)
  {
    Ogre::MovableObject *rotationObj = dPtr->rotationNode->getAttachedObject(0);
    if (rotationObj)
    {
      rotationObj->setVisibilityFlags(GZ_VISIBILITY_GUI);
      dynamic_cast<Ogre::Entity *>(rotationObj)->setMaterialName(
          this->GetMaterialName());
    }
    dPtr->rotationNode->setVisible(this->GetVisible());
    dPtr->sceneNode->addChild(dPtr->rotationNode);
  }
  else
  {
    dPtr->sceneNode->removeChild(dPtr->rotationNode);
  }
}
