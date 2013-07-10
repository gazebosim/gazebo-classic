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
  this->state = MANIP_NONE;
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

  this->CreateRotateVisual();
  this->CreateTranslateVisual();
  this->CreateScaleVisual();

  this->SetVisibilityFlags(GZ_VISIBILITY_GUI);

//  this->SetVisible(false);
  this->transVisual->SetVisible(false);
  this->rotVisual->SetVisible(false);
  this->scaleVisual->SetVisible(false);

  this->GetSceneNode()->setInheritScale(false);
}

/////////////////////////////////////////////////
void Manipulator::Attach(rendering::VisualPtr _vis)
{
  if (this->parent)
    this->parent->DetachVisual(shared_from_this());
  this->parent = _vis;
  this->parent->AttachVisual(shared_from_this());
  this->SetPosition(math::Vector3(0, 0, 0));

  /// TODO set scale of visuals to be size of vis bounding box
  /// Commented out because it doesn't look nice
  // math::Vector3 bboxSize = _vis->GetBoundingBox().GetSize()
  //    * _vis->GetScale();
  // double max = std::max(std::max(bboxSize.x, bboxSize.y), bboxSize.z);
  // this->SetScale(math::Vector3(max, max, max));
}

/////////////////////////////////////////////////
void Manipulator::Detach()
{
  if (this->parent)
    this->parent->DetachVisual(shared_from_this());
  this->parent.reset();
}

/////////////////////////////////////////////////
void Manipulator::SetMode(const std::string &_mode)
{
  ManipulationMode tmpMode = MANIP_NONE;

  if (_mode == "translate")
  {
    tmpMode = TRANS;
  }
  else if (_mode == "rotate")
  {
    tmpMode = ROT;
  }
  else if (_mode == "scale")
  {
    tmpMode = SCALE;
  }
  else if (_mode == "universal")
  {
    tmpMode = TRANS_ROT;
  }

  this->SetMode(tmpMode);
}

/////////////////////////////////////////////////
void Manipulator::SetMode(ManipulationMode _mode)
{
  if (_mode == this->mode)
    return;

  this->mode = _mode;

  this->transVisual->SetVisible(false);
  this->rotVisual->SetVisible(false);
  this->scaleVisual->SetVisible(false);

  if (this->mode == TRANS)
    this->transVisual->SetVisible(true);
  else if (this->mode == ROT)
    this->rotVisual->SetVisible(true);
  else if (this->mode == SCALE)
    this->scaleVisual->SetVisible(true);
  else if (this->mode == TRANS_ROT)
  {
    this->transVisual->SetVisible(true);
    this->rotVisual->SetVisible(true);
  }
}

/*
/////////////////////////////////////////////////
void Manipulator::SetHighlight(const std::string &_mode)
{
  if (this->state == _mode)
    return;

  this->state = _mode;

  if (this->activeVis)
  {
    this->activeVis->SetMaterial(
        this->highlightMaterials[this->activeVis->GetName()]);
  }

  if (this->state == "trans_x")
  {
    this->transXVisual->SetMaterial("Gazebo/RedTransparentLow");
    this->activeVis = this->transXVisual;
  }
  else if (this->state == "trans_y")
  {
    this->transYVisual->SetMaterial("Gazebo/GreenTransparentLow");
    this->activeVis = this->transYVisual;
  }
  else if (this->state == "trans_z")
  {
    this->transZVisual->SetMaterial("Gazebo/BlueTransparentLow");
    this->activeVis = this->transZVisual;
  }
  else if (this->state == "rot_x")
  {
    this->rotXVisual->SetMaterial("Gazebo/RedTransparentLow");
    this->activeVis = this->rotXVisual;
  }
  else if (this->state == "rot_y")
  {
    this->rotYVisual->SetMaterial("Gazebo/GreenTransparentLow");
    this->activeVis = this->rotYVisual;
  }
  else if (this->state == "rot_z")
  {
    this->rotZVisual->SetMaterial("Gazebo/BlueTransparentLow");
    this->activeVis = this->rotZVisual;
  }
}
*/

/////////////////////////////////////////////////
Manipulator::ManipulationMode Manipulator::GetMode()
{
  return this->mode;
}


/////////////////////////////////////////////////
void Manipulator::SetState(const std::string &_state)
{
  ManipulationMode tmpState = MANIP_NONE;

  if (_state == "trans_x")
  {
    tmpState = TRANS_X;
  }
  else if (_state == "trans_y")
  {
    tmpState = TRANS_Y;
  }
  else if (_state == "trans_z")
  {
    tmpState = TRANS_Z;
  }
  else if (_state == "rot_x")
  {
    tmpState = ROT_X;
  }
  else if (_state == "rot_y")
  {
    tmpState = ROT_Y;
  }
  else if (_state == "rot_z")
  {
    tmpState = ROT_Z;
  }
  else if (_state == "scale_x")
  {
    tmpState = SCALE_X;
  }
  else if (_state == "scale_y")
  {
    tmpState = SCALE_Y;
  }
  else if (_state == "scale_z")
  {
    tmpState = SCALE_Z;
  }
  this->SetState(tmpState);
}

/////////////////////////////////////////////////
void Manipulator::SetState(ManipulationMode _state)
{
  if (this->state == _state)
    return;

  this->state = _state;

  if (this->activeVis)
  {
    this->activeVis->SetMaterial(
        this->highlightMaterials[this->activeVis->GetName()]);
  }

  if (this->state == TRANS_X)
  {
    this->transXVisual->SetMaterial("Gazebo/RedTransparentLow");
    this->activeVis = this->transXVisual;
  }
  else if (this->state == TRANS_Y)
  {
    this->transYVisual->SetMaterial("Gazebo/GreenTransparentLow");
    this->activeVis = this->transYVisual;
  }
  else if (this->state == TRANS_Z)
  {
    this->transZVisual->SetMaterial("Gazebo/BlueTransparentLow");
    this->activeVis = this->transZVisual;
  }
  else if (this->state == ROT_X)
  {
    this->rotXVisual->SetMaterial("Gazebo/RedTransparentLow");
    this->activeVis = this->rotXVisual;
  }
  else if (this->state == ROT_Y)
  {
    this->rotYVisual->SetMaterial("Gazebo/GreenTransparentLow");
    this->activeVis = this->rotYVisual;
  }
  else if (this->state == ROT_Z)
  {
    this->rotZVisual->SetMaterial("Gazebo/BlueTransparentLow");
    this->activeVis = this->rotZVisual;
  }
  else if (this->state == SCALE_X)
  {
    this->scaleXVisual->SetMaterial("Gazebo/RedTransparentLow");
    this->activeVis = this->scaleXVisual;
  }
  else if (this->state == SCALE_Y)
  {
    this->scaleYVisual->SetMaterial("Gazebo/GreenTransparentLow");
    this->activeVis = this->scaleYVisual;
  }
  else if (this->state == SCALE_Z)
  {
    this->scaleZVisual->SetMaterial("Gazebo/BlueTransparentLow");
    this->activeVis = this->scaleZVisual;
  }
}

/////////////////////////////////////////////////
Manipulator::ManipulationMode Manipulator::GetState()
{
  return this->state;
}

/////////////////////////////////////////////////
void Manipulator::CreateTranslateVisual()
{
  // Translation mainipulation tool
  this->transVisual.reset(new rendering::Visual(
      this->name + "__SELECTION_OBJ_TRANS__",
      shared_from_this()));
//      this->rotVisual));

  this->transXVisual.reset(
      new rendering::Visual(
      "__SELECTION_OBJ_TRANS_X__" + this->name, this->transVisual));
  this->transYVisual.reset(
      new rendering::Visual(
      "__SELECTION_OBJ_TRANS_Y__" + this->name, this->transVisual));
  this->transZVisual.reset(
      new rendering::Visual(
      "__SELECTION_OBJ_TRANS_Z__" + this->name, this->transVisual));

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
  transShaftXNode->setScale(0.5, 0.5, 1.0);
  transShaftXNode->setPosition(0, 0, 0.1);
  transHeadXNode->attachObject(headXObj);
  transHeadXNode->setScale(0.5, 0.5, 0.5);
  transHeadXNode->setPosition(0, 0, 0.22);
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
  transShaftYNode->setScale(0.5, 0.5, 1.0);
  transShaftYNode->setPosition(0, 0, 0.1);
  transHeadYNode->attachObject(headYObj);
  transHeadYNode->setScale(0.5, 0.5, 0.5);
  transHeadYNode->setPosition(0, 0, 0.22);
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
  transShaftZNode->setScale(0.5, 0.5, 1.0);
  transShaftZNode->setPosition(0, 0, 0.1);
  transHeadZNode->attachObject(headZObj);
  transHeadZNode->setScale(0.5, 0.5, 0.5);
  transHeadZNode->setPosition(0, 0, 0.22);
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

}

/////////////////////////////////////////////////
void Manipulator::CreateRotateVisual()
{
  // Rotation mainipulation tool
  this->rotVisual.reset(new rendering::Visual(
      this->name + "__SELECTION_OBJ_ROT__",
      shared_from_this()));

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
  rotXObj->setUserAny(Ogre::Any(std::string("rot_x")));

  Ogre::MovableObject *rotYObj =
      (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_ROT_Y__" + this->name, "selection_tube"));
  Ogre::SceneNode *yNode =
      this->rotYVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ__ROT_NODE_Y__"  + this->name);
  yNode->attachObject(rotYObj);
  rotYObj->setUserAny(Ogre::Any(std::string("rot_y")));

  Ogre::MovableObject *rotZObj =
      (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_ROT_Z__" + this->name, "selection_tube"));
  Ogre::SceneNode *zNode =
      this->rotZVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ__ROT_NODE_Z__"  + this->name);
  zNode->attachObject(rotZObj);
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

  // Add to scene so they are selectable by the mouse
  this->scene->AddVisual(this->rotXVisual);
  this->scene->AddVisual(this->rotYVisual);
  this->scene->AddVisual(this->rotZVisual);
}

/////////////////////////////////////////////////
void Manipulator::CreateScaleVisual()
{
  // Scale mainipulation tool
  this->scaleVisual.reset(new rendering::Visual(
      this->name + "__SELECTION_OBJ_SCALE__",
      shared_from_this()));
//      this->rotVisual));

  this->scaleXVisual.reset(
      new rendering::Visual(
      "__SELECTION_OBJ_SCALE_X__" + this->name, this->scaleVisual));
  this->scaleYVisual.reset(
      new rendering::Visual(
      "__SELECTION_OBJ_SCALE_Y__" + this->name, this->scaleVisual));
  this->scaleZVisual.reset(
      new rendering::Visual(
      "__SELECTION_OBJ_SCALE_Z__" + this->name, this->scaleVisual));

  this->scaleXVisual->Load();
  this->scaleYVisual->Load();
  this->scaleZVisual->Load();

  this->InsertMesh("unit_box");

  Ogre::MovableObject *scaleShaftXObj =
      (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_SCALE_SHAFT_X__" + this->name, "axis_shaft"));
  Ogre::MovableObject *scaleHeadXObj =
      (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_SCALE_HEAD_X__" + this->name, "unit_box"));
  Ogre::SceneNode *scaleShaftXNode =
      this->scaleXVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ__SCALE_SHAFT_NODE_X__"  + this->name);
  Ogre::SceneNode *scaleHeadXNode =
      this->scaleXVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ__SCALE_HEAD_NODE_X__"  + this->name);
  scaleShaftXNode->attachObject(scaleShaftXObj);
  scaleShaftXNode->setScale(0.5, 0.5, 1.0);
  scaleShaftXNode->setPosition(0, 0, 0.1);
  scaleHeadXNode->attachObject(scaleHeadXObj);
  scaleHeadXNode->setScale(0.02, 0.02, 0.02);
  scaleHeadXNode->setPosition(0, 0, 0.21);
  scaleShaftXObj->setUserAny(Ogre::Any(std::string("scale_x")));
  scaleHeadXObj->setUserAny(Ogre::Any(std::string("scale_x")));


  Ogre::MovableObject *scaleShaftYObj =
      (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_SCALE_SHAFT_Y__" + this->name, "axis_shaft"));
  Ogre::MovableObject *scaleHeadYObj =
      (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_SCALE_HEAD_Y__" + this->name, "unit_box"));
  Ogre::SceneNode *scaleShaftYNode =
      this->scaleYVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ_SCALE_SHAFT_NODE_Y__"  + this->name);
  Ogre::SceneNode *scaleHeadYNode =
      this->scaleYVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ_SCALE_HEAD_NODE_Y__"  + this->name);
  scaleShaftYNode->attachObject(scaleShaftYObj);
  scaleShaftYNode->setScale(0.5, 0.5, 1.0);
  scaleShaftYNode->setPosition(0, 0, 0.1);
  scaleHeadYNode->attachObject(scaleHeadYObj);
  scaleHeadYNode->setScale(0.02, 0.02, 0.02);
  scaleHeadYNode->setPosition(0, 0, 0.21);
  scaleShaftYObj->setUserAny(Ogre::Any(std::string("scale_y")));
  scaleHeadYObj->setUserAny(Ogre::Any(std::string("scale_y")));

  Ogre::MovableObject *scaleShaftZObj =
      (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_SCALE_SHAFT_Z__" + this->name, "axis_shaft"));
  Ogre::MovableObject *scaleHeadZObj =
      (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_SCALE_HEAD_Z__" + this->name, "unit_box"));
  Ogre::SceneNode *scaleShaftZNode =
      this->scaleZVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ_SCALE_SHAFT_NODE_Z__"  + this->name);
  Ogre::SceneNode *scaleHeadZNode =
      this->scaleZVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ_SCALE_HEAD_NODE_Z__"  + this->name);
  scaleShaftZNode->attachObject(scaleShaftZObj);
  scaleShaftZNode->setScale(0.5, 0.5, 1.0);
  scaleShaftZNode->setPosition(0, 0, 0.1);
  scaleHeadZNode->attachObject(scaleHeadZObj);
  scaleHeadZNode->setScale(0.02, 0.02, 0.02);
  scaleHeadZNode->setPosition(0, 0, 0.21);
  scaleShaftZObj->setUserAny(Ogre::Any(std::string("scale_z")));
  scaleHeadZObj->setUserAny(Ogre::Any(std::string("scale_z")));

  this->scaleXVisual->SetRotation(
      math::Quaternion(math::Vector3(0, 1, 0), GZ_DTOR(90)));
  this->scaleYVisual->SetRotation(
      math::Quaternion(math::Vector3(1, 0, 0), GZ_DTOR(-90)));

  this->highlightMaterials[this->scaleXVisual->GetName()]
      = "Gazebo/RedTransparent";
  this->highlightMaterials[this->scaleYVisual->GetName()]
      = "Gazebo/GreenTransparent";
  this->highlightMaterials[this->scaleZVisual->GetName()]
      = "Gazebo/BlueTransparent";

  this->scaleXVisual->SetMaterial(
      this->highlightMaterials[this->scaleXVisual->GetName()]);
  this->scaleYVisual->SetMaterial(
      this->highlightMaterials[this->scaleYVisual->GetName()]);
  this->scaleZVisual->SetMaterial(
      this->highlightMaterials[this->scaleZVisual->GetName()]);

  this->scaleVisual->SetScale(math::Vector3(5.0, 5.0, 5.0));

  this->scaleXVisual->SetVisibilityFlags(GZ_VISIBILITY_GUI);
  this->scaleYVisual->SetVisibilityFlags(GZ_VISIBILITY_GUI);
  this->scaleZVisual->SetVisibilityFlags(GZ_VISIBILITY_GUI);

  // Add to scene so they are selectable by the mouse
  this->scene->AddVisual(this->scaleXVisual);
  this->scene->AddVisual(this->scaleYVisual);
  this->scene->AddVisual(this->scaleZVisual);
}
