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

#include "gazebo/gui/Gui.hh"

#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/AxisVisual.hh"
#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Manipulator.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
Manipulator::Manipulator(const std::string &_name, VisualPtr _vis)
  : Visual(_name, _vis, false)
{
  static int counter = 0;
  std::stringstream ss;
  ss << "__MANIP__" + counter++;
  this->name = ss.str();
  this->mode = MANIP_NONE;
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
  Visual::Load();

  // Rotation mainipulation tool
  this->rotVisual.reset(new rendering::Visual(
      this->name + "__SELECTION_OBJ_ROT__",
      shared_from_this()));
//      this->manipVisual));
  this->transVisual.reset(new rendering::Visual(
      this->name + "__SELECTION_OBJ_TRANS__",
      this->rotVisual));
  this->scaleVisual.reset(new rendering::Visual(
      this->name + "__SELECTION_OBJ_SCALE__",
      this->transVisual));

  this->rotXVisual.reset(
      new rendering::Visual(
      this->name + "__SELECTION_OBJ_ROT_X__", this->rotVisual));
  this->rotYVisual.reset(
      new rendering::Visual(
      this->name + "__SELECTION_OBJ_ROT_Y__", this->rotVisual));
  this->rotZVisual.reset(
      new rendering::Visual(
      this->name + "__SELECTION_OBJ_ROT_Z__", this->rotVisual));

  this->rotVisual->InsertMesh("selection_tube");

  Ogre::MovableObject *rotXObj =
      (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_ROT_X__" + this->name, "selection_tube"));
  Ogre::SceneNode *xNode =
      this->rotXVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ__ROT_NODE_X__"  + this->name);
  xNode->attachObject(rotXObj);
//  rotXObj->setUserAny(Ogre::Any(this->rotXVisual->GetName()));
  rotXObj->setUserAny(Ogre::Any(std::string("rot_x")));


  Ogre::MovableObject *rotYObj =
      (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_ROT_Y__" + this->name, "selection_tube"));
  Ogre::SceneNode *yNode =
      this->rotYVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ__ROT_NODE_Y__"  + this->name);
  yNode->attachObject(rotYObj);
//  rotYObj->setUserAny(Ogre::Any(this->rotYVisual->GetName()));
  rotYObj->setUserAny(Ogre::Any(std::string("rot_y")));

  Ogre::MovableObject *rotZObj =
      (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_ROT_Z__" + this->name, "selection_tube"));
  Ogre::SceneNode *zNode =
      this->rotZVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ__ROT_NODE_Z__"  + this->name);
  zNode->attachObject(rotZObj);
//  rotZObj->setUserAny(Ogre::Any(this->rotZVisual->GetName()));
  rotZObj->setUserAny(Ogre::Any(std::string("rot_z")));

  this->rotXVisual->Load();
  this->rotYVisual->Load();
  this->rotZVisual->Load();

  this->rotXVisual->SetRotation(
      math::Quaternion(math::Vector3(0, 1, 0), GZ_DTOR(90)));
  this->rotYVisual->SetRotation(
      math::Quaternion(math::Vector3(1, 0, 0), GZ_DTOR(-90)));

  this->highlightMaterials[this->rotXVisual->GetName()]
      = "Gazebo/RedTransparent";
  this->highlightMaterials[this->rotYVisual->GetName()]
      = "Gazebo/GreenTransparent";
  this->highlightMaterials[this->rotZVisual->GetName()]
      = "Gazebo/BlueTransparent";

  this->rotXVisual->SetMaterial(
      this->highlightMaterials[this->rotXVisual->GetName()]);
  this->rotYVisual->SetMaterial(
      this->highlightMaterials[this->rotYVisual->GetName()]);
  this->rotZVisual->SetMaterial(
      this->highlightMaterials[this->rotZVisual->GetName()]);

  this->rotVisual->SetScale(math::Vector3(1.0, 1.0, 1.0));

  this->rotXVisual->SetVisibilityFlags(GZ_VISIBILITY_GUI);
  this->rotYVisual->SetVisibilityFlags(GZ_VISIBILITY_GUI);
  this->rotZVisual->SetVisibilityFlags(GZ_VISIBILITY_GUI);

  // Rotation mainipulation tool
  this->transXVisual.reset(
      new rendering::Visual(
      "__SELECTION_OBJ_ROT_X__" + this->name, this->transVisual));
  this->transYVisual.reset(
      new rendering::Visual(
      "__SELECTION_OBJ_ROT_Y__" + this->name, this->transVisual));
  this->transZVisual.reset(
      new rendering::Visual(
      "__SELECTION_OBJ_ROT_Z__" + this->name, this->transVisual));

  this->transXVisual->Load();
  this->transYVisual->Load();
  this->transZVisual->Load();

  this->InsertMesh("axis_shaft");
  this->InsertMesh("axis_head");

  Ogre::MovableObject *shaftXObj =
      (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_TRANS_SHAFT_X__" + this->name, "axis_shaft"));
  Ogre::MovableObject *headXObj =
      (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_TRANS_HEAD_X__" + this->name, "axis_head"));
  Ogre::SceneNode *transShaftXNode =
      this->transXVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ__TRANS_SHAFT_NODE_X__"  + this->name);
  Ogre::SceneNode *transHeadXNode =
      this->transXVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ__TRANS_HEAD_NODE_X__"  + this->name);
  transShaftXNode->attachObject(shaftXObj);
  transShaftXNode->setPosition(0, 0, 0.1);
  transHeadXNode->attachObject(headXObj);
  transHeadXNode->setPosition(0, 0, 0.24);
//  shaftXObj->setUserAny(Ogre::Any(this->transXVisual->GetName()));
//  headXObj->setUserAny(Ogre::Any(this->transXVisual->GetName()));
  shaftXObj->setUserAny(Ogre::Any(std::string("trans_x")));
  headXObj->setUserAny(Ogre::Any(std::string("trans_x")));


  Ogre::MovableObject *shaftYObj =
      (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_TRANS_SHAFT_Y__" + this->name, "axis_shaft"));
  Ogre::MovableObject *headYObj =
      (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_TRANS_HEAD_Y__" + this->name, "axis_head"));
  Ogre::SceneNode *transShaftYNode =
      this->transYVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ_TRANS_SHAFT_NODE_Y__"  + this->name);
  Ogre::SceneNode *transHeadYNode =
      this->transYVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ_TRANS_HEAD_NODE_Y__"  + this->name);
  transShaftYNode->attachObject(shaftYObj);
  transShaftYNode->setPosition(0, 0, 0.1);
  transHeadYNode->attachObject(headYObj);
  transHeadYNode->setPosition(0, 0, 0.24);
//  shaftYObj->setUserAny(Ogre::Any(this->transYVisual->GetName()));
//  headYObj->setUserAny(Ogre::Any(this->transYVisual->GetName()));
  shaftYObj->setUserAny(Ogre::Any(std::string("trans_y")));
  headYObj->setUserAny(Ogre::Any(std::string("trans_y")));

  Ogre::MovableObject *shaftZObj =
      (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_TRANS_SHAFT_Z__" + this->name, "axis_shaft"));
  Ogre::MovableObject *headZObj =
      (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_TRANS_HEAD_Z__" + this->name, "axis_head"));
  Ogre::SceneNode *transShaftZNode =
      this->transZVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ_TRANS_SHAFT_NODE_Z__"  + this->name);
  Ogre::SceneNode *transHeadZNode =
      this->transZVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ_TRANS_HEAD_NODE_Z__"  + this->name);
  transShaftZNode->attachObject(shaftZObj);
  transShaftZNode->setPosition(0, 0, 0.1);
  transHeadZNode->attachObject(headZObj);
  transHeadZNode->setPosition(0, 0, 0.24);
//  shaftZObj->setUserAny(Ogre::Any(this->transZVisual->GetName()));
//  headZObj->setUserAny(Ogre::Any(this->transZVisual->GetName()));
  shaftZObj->setUserAny(Ogre::Any(std::string("trans_z")));
  headZObj->setUserAny(Ogre::Any(std::string("trans_z")));

  this->transXVisual->SetRotation(
      math::Quaternion(math::Vector3(0, 1, 0), GZ_DTOR(90)));
  this->transYVisual->SetRotation(
      math::Quaternion(math::Vector3(1, 0, 0), GZ_DTOR(-90)));

  this->highlightMaterials[this->transXVisual->GetName()]
      = "Gazebo/RedTransparent";
  this->highlightMaterials[this->transYVisual->GetName()]
      = "Gazebo/GreenTransparent";
  this->highlightMaterials[this->transZVisual->GetName()]
      = "Gazebo/BlueTransparent";

  this->transXVisual->SetMaterial(
      this->highlightMaterials[this->transXVisual->GetName()]);
  this->transYVisual->SetMaterial(
      this->highlightMaterials[this->transYVisual->GetName()]);
  this->transZVisual->SetMaterial(
      this->highlightMaterials[this->transZVisual->GetName()]);

  this->transVisual->SetScale(math::Vector3(5.0, 5.0, 5.0));

  this->transXVisual->SetVisibilityFlags(GZ_VISIBILITY_GUI);
  this->transYVisual->SetVisibilityFlags(GZ_VISIBILITY_GUI);
  this->transZVisual->SetVisibilityFlags(GZ_VISIBILITY_GUI);

  // Add to scene so they are selectable by the mouse
  this->scene->AddVisual(this->transXVisual);
  this->scene->AddVisual(this->transYVisual);
  this->scene->AddVisual(this->transZVisual);
  this->scene->AddVisual(this->rotXVisual);
  this->scene->AddVisual(this->rotYVisual);
  this->scene->AddVisual(this->rotZVisual);

  this->SetVisibilityFlags(GZ_VISIBILITY_GUI);

  this->SetVisible(false);
}

/////////////////////////////////////////////////
void Manipulator::Attach(rendering::VisualPtr _vis)
{
  if (this->parent)
    this->parent->DetachVisual(shared_from_this());
  this->parent = _vis;
  this->parent->AttachVisual(shared_from_this());
  this->SetPosition(math::Vector3(0, 0, 0));

  ///TODO set scale of visuals to be size of vis bounding box
}
/*
/////////////////////////////////////////////////
void Manipulator::Attach(std::string _entity)
{
}*/

/////////////////////////////////////////////////
void Manipulator::Detach()
{
  if (this->parent)
    this->parent->DetachVisual(shared_from_this());
  this->parent.reset();
}
/*
/////////////////////////////////////////////////
void Manipulator::SetMode(const std::string &_mode)
{
  if (_mode == this->mode)
    return;

  if (_mode == "trans_x")
  {
    gzerr << " trans x " << std::endl;
  }
  else if (_mode == "trans_y")
  {
    gzerr << " trans y " << std::endl;
  }
  else if (_mode == "trans_z")
  {
    gzerr << " trans z " << std::endl;
  }
  else if (_mode == "rot_x")
  {
    gzerr << " rot x " << std::endl;
  }
  else if (_mode == "rot_y")
  {
    gzerr << " rot y " << std::endl;
  }
  else if (_mode == "rot_z")
  {
    gzerr << " rot z " << std::endl;
  }
}*/
/*
/////////////////////////////////////////////////
void Manipulator::SetHighlight(const std::string &_mode)
{
  if (this->mode == _mode)
    return;

  this->mode = _mode;

  if (this->activeVis)
  {
    this->activeVis->SetMaterial(
        this->highlightMaterials[this->activeVis->GetName()]);
  }

  if (this->mode == "trans_x")
  {
    this->transXVisual->SetMaterial("Gazebo/RedTransparentLow");
    this->activeVis = this->transXVisual;
  }
  else if (this->mode == "trans_y")
  {
    this->transYVisual->SetMaterial("Gazebo/GreenTransparentLow");
    this->activeVis = this->transYVisual;
  }
  else if (this->mode == "trans_z")
  {
    this->transZVisual->SetMaterial("Gazebo/BlueTransparentLow");
    this->activeVis = this->transZVisual;
  }
  else if (this->mode == "rot_x")
  {
    this->rotXVisual->SetMaterial("Gazebo/RedTransparentLow");
    this->activeVis = this->rotXVisual;
  }
  else if (this->mode == "rot_y")
  {
    this->rotYVisual->SetMaterial("Gazebo/GreenTransparentLow");
    this->activeVis = this->rotYVisual;
  }
  else if (this->mode == "rot_z")
  {
    this->rotZVisual->SetMaterial("Gazebo/BlueTransparentLow");
    this->activeVis = this->rotZVisual;
  }
}

/////////////////////////////////////////////////
std::string Manipulator::GetMode() const
{
  return this->mode;
}*/

/////////////////////////////////////////////////
void Manipulator::SetHighlight(const std::string &_mode)
{
  ManipulationMode tmpMode = MANIP_NONE;

  if (_mode == "trans_x")
  {
    tmpMode = TRANS_X;
  }
  else if (_mode == "trans_y")
  {
    tmpMode = TRANS_Y;
  }
  else if (_mode == "trans_z")
  {
    tmpMode = TRANS_Z;
  }
  else if (_mode == "rot_x")
  {
    tmpMode = ROT_X;
  }
  else if (_mode == "rot_y")
  {
    tmpMode = ROT_Y;
  }
  else if (_mode == "rot_z")
  {
    tmpMode = ROT_Z;
  }
  this->SetHighlight(tmpMode);
}

/////////////////////////////////////////////////
void Manipulator::SetHighlight(ManipulationMode _mode)
{
  if (this->mode == _mode)
    return;

  this->mode = _mode;

  if (this->activeVis)
  {
    this->activeVis->SetMaterial(
        this->highlightMaterials[this->activeVis->GetName()]);
  }

  if (this->mode == TRANS_X)
  {
    this->transXVisual->SetMaterial("Gazebo/RedTransparentLow");
    this->activeVis = this->transXVisual;
  }
  else if (this->mode == TRANS_Y)
  {
    this->transYVisual->SetMaterial("Gazebo/GreenTransparentLow");
    this->activeVis = this->transYVisual;
  }
  else if (this->mode == TRANS_Z)
  {
    this->transZVisual->SetMaterial("Gazebo/BlueTransparentLow");
    this->activeVis = this->transZVisual;
  }
  else if (this->mode == ROT_X)
  {
    this->rotXVisual->SetMaterial("Gazebo/RedTransparentLow");
    this->activeVis = this->rotXVisual;
  }
  else if (this->mode == ROT_Y)
  {
    this->rotYVisual->SetMaterial("Gazebo/GreenTransparentLow");
    this->activeVis = this->rotYVisual;
  }
  else if (this->mode == ROT_Z)
  {
    this->rotZVisual->SetMaterial("Gazebo/BlueTransparentLow");
    this->activeVis = this->rotZVisual;
  }
}

/////////////////////////////////////////////////
Manipulator::ManipulationMode Manipulator::GetMode()
{
  return this->mode;
}
