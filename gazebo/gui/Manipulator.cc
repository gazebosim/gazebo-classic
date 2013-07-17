/*
 * Copyright 2012 Open Source Robotics Foundation
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

#include "gazebo/common/MeshManager.hh"

#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/AxisVisual.hh"
#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/Scene.hh"

#include "gazebo/gui/Gui.hh"
#include "gazebo/gui/Manipulator.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
Manipulator::Manipulator()
{
  static int counter = 0;
  std::stringstream ss;
  ss << "__MANIP__" + counter++;
  this->name = ss.str();
}

/////////////////////////////////////////////////
Manipulator::~Manipulator()
{
  this->axisVisual.reset();
  this->parent.reset();
  this->manipVisual.reset();
}

/////////////////////////////////////////////////
void Manipulator::Load()
{
  if (!gui::get_active_camera() ||
      !gui::get_active_camera()->GetScene())
  {
    gzerr << "No camera or scene found" << std::endl;
    return;
  }

  rendering::ScenePtr scene = gui::get_active_camera()->GetScene();

  this->manipVisual.reset(new rendering::Visual(this->name + "__MANIP__",
      scene->GetWorldVisual()));
  this->parent = scene->GetWorldVisual();

//  this->rotVisual->InsertMesh("selection_tube");
  this->rotVisual.reset(new rendering::Visual(this->name + "__ROT__",
      this->manipVisual));
  this->transVisual.reset(new rendering::Visual(this->name + "__TRANS__",
      this->rotVisual));
  this->scaleVisual.reset(new rendering::Visual(this->name + "__SCALE__",
      this->transVisual));

  this->rotXVisual.reset(
      new rendering::Visual(this->name + "__ROT_X__", this->rotVisual));
  this->rotYVisual.reset(
      new rendering::Visual(this->name + "__ROT_Y__", this->rotVisual));
  this->rotZVisual.reset(
      new rendering::Visual(this->name + "__ROT_Z__", this->rotVisual));

  this->rotVisual->InsertMesh("selection_tube");
  Ogre::MovableObject *rotXObj =
    (Ogre::MovableObject*)(scene->GetManager()->createEntity(
          this->name + "__ROT_X__", "selection_tube"));
  Ogre::SceneNode *xNode =
      this->rotXVisual->GetSceneNode()->createChildSceneNode(
      this->name + "__ROT_NODE_X__");
  xNode->attachObject(rotXObj);
  Ogre::MovableObject *rotYObj =
    (Ogre::MovableObject*)(scene->GetManager()->createEntity(
          this->name + "__ROT_Y__", "selection_tube"));
  Ogre::SceneNode *yNode =
      this->rotYVisual->GetSceneNode()->createChildSceneNode(
      this->name + "__ROT_NODE_Y__");
  yNode->attachObject(rotYObj);
  Ogre::MovableObject *rotZObj =
    (Ogre::MovableObject*)(scene->GetManager()->createEntity(
          this->name + "__ROT_Z__", "selection_tube"));
  Ogre::SceneNode *zNode =
      this->rotZVisual->GetSceneNode()->createChildSceneNode(
      this->name + "__ROT_NODE_Z__");
  zNode->attachObject(rotZObj);

/*
  this->rotXVisual->AttachMesh("selection_tube");
  this->rotYVisual->AttachMesh("selection_tube");
  this->rotZVisual->AttachMesh("selection_tube");
*/

  this->rotXVisual->Load();
  this->rotYVisual->Load();
  this->rotZVisual->Load();

  this->rotXVisual->SetRotation(
      math::Quaternion(math::Vector3(0, 1, 0), GZ_DTOR(90)));
  this->rotYVisual->SetRotation(
      math::Quaternion(math::Vector3(1, 0, 0), GZ_DTOR(-90)));

  this->rotXVisual->SetMaterial("__GAZEBO_TRANS_RED_MATERIAL__");
  this->rotYVisual->SetMaterial("__GAZEBO_TRANS_GREEN_MATERIAL__");
  this->rotZVisual->SetMaterial("__GAZEBO_TRANS_BLUE_MATERIAL__");

  this->rotVisual->SetScale(math::Vector3(0.5, 0.5, 0.5));

  this->axisVisual.reset(
      new rendering::AxisVisual(this->name + "_AXIS", this->transVisual));
  this->axisVisual->Load();

  this->axisVisual->SetScale(math::Vector3(4.0, 4.0, 4.0));


  //this->SetVisibilityFlags(GZ_VISIBILITY_GUI);


}

/////////////////////////////////////////////////
void Manipulator::Attach(rendering::VisualPtr _vis)
{
  this->parent->DetachVisual(shared_from_this());
  this->parent = _vis;
  this->parent->AttachVisual(shared_from_this());
}

/////////////////////////////////////////////////
void Manipulator::Attach(std::string _entity)
{
}
