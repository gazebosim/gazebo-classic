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

#include "gazebo/common/MeshManager.hh"

#include "gazebo/gui/GuiIface.hh"

#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/SelectionObj.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
SelectionObj::SelectionObj(const std::string &_name, VisualPtr _vis)
  : Visual(_name, _vis, false)
{
  this->state = SELECTION_NONE;
  this->mode = SELECTION_NONE;

  this->maxScale = 3.0;
  this->minScale = 0.5;

  this->xAxisMatOverlay = "Gazebo/RedTransparentOverlay";
  this->yAxisMatOverlay = "Gazebo/GreenTransparentOverlay";
  this->zAxisMatOverlay = "Gazebo/BlueTransparentOverlay";

  this->xAxisMat = "Gazebo/RedTransparent";
  this->yAxisMat = "Gazebo/GreenTransparent";
  this->zAxisMat = "Gazebo/BlueTransparent";
}

/////////////////////////////////////////////////
SelectionObj::~SelectionObj()
{
  this->parent.reset();
}

/////////////////////////////////////////////////
void SelectionObj::Load()
{
  Visual::Load();

  this->CreateRotateVisual();
  this->CreateTranslateVisual();
  this->CreateScaleVisual();

  this->SetVisibilityFlags(GZ_VISIBILITY_GUI);

  this->transVisual->SetVisible(false);
  this->rotVisual->SetVisible(false);
  this->scaleVisual->SetVisible(false);

  this->GetSceneNode()->setInheritScale(false);
}

/////////////////////////////////////////////////
void SelectionObj::Attach(rendering::VisualPtr _vis)
{
  if (this->parent)
  {
    if (this->parent == _vis)
      return;
    this->parent->DetachVisual(shared_from_this());
  }

  this->parent = _vis;
  this->parent->AttachVisual(shared_from_this());
  this->SetPosition(math::Vector3(0, 0, 0));

  this->UpdateSize();
}

/////////////////////////////////////////////////
void SelectionObj::UpdateSize()
{
  VisualPtr vis = this->parent;

  // don't include the selection obj itself when calculating the size.
  this->Detach();
  math::Vector3 bboxSize = vis->GetBoundingBox().GetSize()
      * vis->GetScale();
  this->parent = vis;
  this->parent->AttachVisual(shared_from_this());

  double max = std::max(std::max(bboxSize.x, bboxSize.y), bboxSize.z);

  max = std::min(std::max(this->minScale, max), this->maxScale);

  // Handle special case for rotation visuals. Only set the visuals to be
  // overlays for big objects.
  if (math::equal(max, this->maxScale))
  {
    this->rotXVisual->SetMaterial(this->xAxisMatOverlay, false);
    this->rotYVisual->SetMaterial(this->yAxisMatOverlay, false);
    this->rotZVisual->SetMaterial(this->zAxisMatOverlay, false);
  }
  else
  {
    this->rotXVisual->SetMaterial(this->xAxisMat, false);
    this->rotYVisual->SetMaterial(this->yAxisMat, false);
    this->rotZVisual->SetMaterial(this->zAxisMat, false);
  }
  this->SetScale(math::Vector3(max, max, max));
}

/////////////////////////////////////////////////
void SelectionObj::Detach()
{
  if (this->parent)
    this->parent->DetachVisual(shared_from_this());
  this->parent.reset();
}

/////////////////////////////////////////////////
void SelectionObj::SetMode(const std::string &_mode)
{
  SelectionMode tmpMode = SELECTION_NONE;

  if (_mode == "translate")
    tmpMode = TRANS;
  else if (_mode == "rotate")
    tmpMode = ROT;
  else if (_mode == "scale")
    tmpMode = SCALE;

  this->SetMode(tmpMode);
}

/////////////////////////////////////////////////
void SelectionObj::SetMode(SelectionMode _mode)
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
}

/////////////////////////////////////////////////
SelectionObj::SelectionMode SelectionObj::GetMode()
{
  return this->mode;
}

/////////////////////////////////////////////////
void SelectionObj::SetState(const std::string &_state)
{
  SelectionMode tmpState = SELECTION_NONE;

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
void SelectionObj::SetState(SelectionMode _state)
{
  if (this->state == _state)
    return;

  this->state = _state;

  if (this->selectedVis)
  {
    Ogre::MaterialPtr mat =
      Ogre::MaterialManager::getSingleton().getByName(
      this->selectedVis->GetMaterialName());
    mat->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setAlphaOperation(
      Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, 0.5);
    this->selectedVis.reset();
  }

  if (this->state == TRANS_X)
    this->selectedVis = this->transXVisual;
  else if (this->state == TRANS_Y)
    this->selectedVis = this->transYVisual;
  else if (this->state == TRANS_Z)
    this->selectedVis = this->transZVisual;
  else if (this->state == ROT_X)
    this->selectedVis = this->rotXVisual;
  else if (this->state == ROT_Y)
    this->selectedVis = this->rotYVisual;
  else if (this->state == ROT_Z)
    this->selectedVis = this->rotZVisual;
  else if (this->state == SCALE_X)
    this->selectedVis = this->scaleXVisual;
  else if (this->state == SCALE_Y)
    this->selectedVis = this->scaleYVisual;
  else if (this->state == SCALE_Z)
    this->selectedVis = this->scaleZVisual;

  if (this->selectedVis)
  {
    Ogre::MaterialPtr mat =
      Ogre::MaterialManager::getSingleton().getByName(
      this->selectedVis->GetMaterialName());
    mat->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setAlphaOperation(
      Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, 0.7);
  }
}

/////////////////////////////////////////////////
void SelectionObj::SetGlobal(bool _global)
{
  this->transVisual->GetSceneNode()->setInheritOrientation(!_global);
  this->rotVisual->GetSceneNode()->setInheritOrientation(!_global);
}

/////////////////////////////////////////////////
SelectionObj::SelectionMode SelectionObj::GetState()
{
  return this->state;
}

/////////////////////////////////////////////////
void SelectionObj::CreateTranslateVisual()
{
  // Translation mainipulation tool
  this->transVisual.reset(new rendering::Visual(
      this->GetName() + "__SELECTION_OBJ_TRANS__",
      shared_from_this()));

  this->transXVisual.reset(
      new rendering::Visual(
      "__SELECTION_OBJ_TRANS_X__" + this->GetName(), this->transVisual));
  this->transYVisual.reset(
      new rendering::Visual(
      "__SELECTION_OBJ_TRANS_Y__" + this->GetName(), this->transVisual));
  this->transZVisual.reset(
      new rendering::Visual(
      "__SELECTION_OBJ_TRANS_Z__" + this->GetName(), this->transVisual));

  this->transXVisual->Load();
  this->transYVisual->Load();
  this->transZVisual->Load();

  this->InsertMesh("axis_shaft");
  this->InsertMesh("axis_head");

  Ogre::MovableObject *shaftXObj =
      (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_TRANS_SHAFT_X__" + this->GetName(), "axis_shaft"));
  Ogre::MovableObject *headXObj =
      (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_TRANS_HEAD_X__" + this->GetName(), "axis_head"));
  Ogre::SceneNode *transShaftXNode =
      this->transXVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ__TRANS_SHAFT_NODE_X__"  + this->GetName());
  Ogre::SceneNode *transHeadXNode =
      this->transXVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ__TRANS_HEAD_NODE_X__"  + this->GetName());
  transShaftXNode->attachObject(shaftXObj);
  transShaftXNode->setScale(0.5, 0.5, 1.0);
  transShaftXNode->setPosition(0, 0, 0.1);
  transHeadXNode->attachObject(headXObj);
  transHeadXNode->setScale(0.5, 0.5, 0.5);
  transHeadXNode->setPosition(0, 0, 0.22);
  shaftXObj->setUserAny(Ogre::Any(std::string("trans_x")));
  headXObj->setUserAny(Ogre::Any(std::string("trans_x")));
  shaftXObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
  headXObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);

  Ogre::MovableObject *shaftYObj =
      (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_TRANS_SHAFT_Y__" + this->GetName(), "axis_shaft"));
  Ogre::MovableObject *headYObj =
      (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_TRANS_HEAD_Y__" + this->GetName(), "axis_head"));
  Ogre::SceneNode *transShaftYNode =
      this->transYVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ_TRANS_SHAFT_NODE_Y__"  + this->GetName());
  Ogre::SceneNode *transHeadYNode =
      this->transYVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ_TRANS_HEAD_NODE_Y__"  + this->GetName());
  transShaftYNode->attachObject(shaftYObj);
  transShaftYNode->setScale(0.5, 0.5, 1.0);
  transShaftYNode->setPosition(0, 0, 0.1);
  transHeadYNode->attachObject(headYObj);
  transHeadYNode->setScale(0.5, 0.5, 0.5);
  transHeadYNode->setPosition(0, 0, 0.22);
  shaftYObj->setUserAny(Ogre::Any(std::string("trans_y")));
  headYObj->setUserAny(Ogre::Any(std::string("trans_y")));
  shaftYObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
  headYObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);

  Ogre::MovableObject *shaftZObj =
      (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_TRANS_SHAFT_Z__" + this->GetName(), "axis_shaft"));
  Ogre::MovableObject *headZObj =
      (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_TRANS_HEAD_Z__" + this->GetName(), "axis_head"));
  Ogre::SceneNode *transShaftZNode =
      this->transZVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ_TRANS_SHAFT_NODE_Z__"  + this->GetName());
  Ogre::SceneNode *transHeadZNode =
      this->transZVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ_TRANS_HEAD_NODE_Z__"  + this->GetName());
  transShaftZNode->attachObject(shaftZObj);
  transShaftZNode->setScale(0.5, 0.5, 1.0);
  transShaftZNode->setPosition(0, 0, 0.1);
  transHeadZNode->attachObject(headZObj);
  transHeadZNode->setScale(0.5, 0.5, 0.5);
  transHeadZNode->setPosition(0, 0, 0.22);
  shaftZObj->setUserAny(Ogre::Any(std::string("trans_z")));
  headZObj->setUserAny(Ogre::Any(std::string("trans_z")));
  shaftZObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
  headZObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);

  this->transXVisual->SetRotation(
      math::Quaternion(math::Vector3(0, 1, 0), GZ_DTOR(90)));
  this->transYVisual->SetRotation(
      math::Quaternion(math::Vector3(1, 0, 0), GZ_DTOR(-90)));

  this->transXVisual->SetMaterial(this->xAxisMatOverlay);
  this->transYVisual->SetMaterial(this->yAxisMatOverlay);
  this->transZVisual->SetMaterial(this->zAxisMatOverlay);

  this->transVisual->SetScale(math::Vector3(5.0, 5.0, 5.0));

  this->transXVisual->SetVisibilityFlags(
      GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE);
  this->transYVisual->SetVisibilityFlags(
      GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE);
  this->transZVisual->SetVisibilityFlags(
      GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE);

  // Add to scene so they are selectable by the mouse
  this->scene->AddVisual(this->transXVisual);
  this->scene->AddVisual(this->transYVisual);
  this->scene->AddVisual(this->transZVisual);
}

/////////////////////////////////////////////////
void SelectionObj::CreateRotateVisual()
{
  // Rotation mainipulation tool
  this->rotVisual.reset(new rendering::Visual(
      this->GetName() + "__SELECTION_OBJ_ROT__",
      shared_from_this()));

  this->rotXVisual.reset(
      new rendering::Visual(
      this->GetName() + "__SELECTION_OBJ_ROT_X__", this->rotVisual));
  this->rotYVisual.reset(
      new rendering::Visual(
      this->GetName() + "__SELECTION_OBJ_ROT_Y__", this->rotVisual));
  this->rotZVisual.reset(
      new rendering::Visual(
      this->GetName() + "__SELECTION_OBJ_ROT_Z__", this->rotVisual));

  this->rotVisual->InsertMesh("selection_tube");

  Ogre::MovableObject *rotXObj =
      (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_ROT_X__" + this->GetName(), "selection_tube"));
  Ogre::SceneNode *xNode =
      this->rotXVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ__ROT_NODE_X__"  + this->GetName());
  xNode->attachObject(rotXObj);
  rotXObj->setUserAny(Ogre::Any(std::string("rot_x")));
  rotXObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);

  Ogre::MovableObject *rotYObj =
      (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_ROT_Y__" + this->GetName(), "selection_tube"));
  Ogre::SceneNode *yNode =
      this->rotYVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ__ROT_NODE_Y__"  + this->GetName());
  yNode->attachObject(rotYObj);
  rotYObj->setUserAny(Ogre::Any(std::string("rot_y")));
  rotYObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);

  Ogre::MovableObject *rotZObj =
      (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_ROT_Z__" + this->GetName(), "selection_tube"));
  Ogre::SceneNode *zNode =
      this->rotZVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ__ROT_NODE_Z__"  + this->GetName());
  zNode->attachObject(rotZObj);
  rotZObj->setUserAny(Ogre::Any(std::string("rot_z")));
  rotZObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);

  this->rotXVisual->Load();
  this->rotYVisual->Load();
  this->rotZVisual->Load();

  this->rotXVisual->SetRotation(
      math::Quaternion(math::Vector3(0, 1, 0), GZ_DTOR(90)));
  this->rotYVisual->SetRotation(
      math::Quaternion(math::Vector3(1, 0, 0), GZ_DTOR(-90)));

  // By default the visuals are not overlays like translation or scale visuals.
  // This is so that the rings does not block the object it's attached too,
  // and also gives with better depth perception.
  this->rotXVisual->SetMaterial(this->xAxisMat);
  this->rotYVisual->SetMaterial(this->yAxisMat);
  this->rotZVisual->SetMaterial(this->zAxisMat);

  this->rotVisual->SetScale(math::Vector3(1.0, 1.0, 1.0));

  this->rotXVisual->SetVisibilityFlags(
      GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE);
  this->rotYVisual->SetVisibilityFlags(
      GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE);
  this->rotZVisual->SetVisibilityFlags(
      GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE);

  // Add to scene so they are selectable by the mouse
  this->scene->AddVisual(this->rotXVisual);
  this->scene->AddVisual(this->rotYVisual);
  this->scene->AddVisual(this->rotZVisual);
}

/////////////////////////////////////////////////
void SelectionObj::CreateScaleVisual()
{
  // Scale mainipulation tool
  this->scaleVisual.reset(new rendering::Visual(
      this->GetName() + "__SELECTION_OBJ_SCALE__",
      shared_from_this()));

  this->scaleXVisual.reset(
      new rendering::Visual(
      "__SELECTION_OBJ_SCALE_X__" + this->GetName(), this->scaleVisual));
  this->scaleYVisual.reset(
      new rendering::Visual(
      "__SELECTION_OBJ_SCALE_Y__" + this->GetName(), this->scaleVisual));
  this->scaleZVisual.reset(
      new rendering::Visual(
      "__SELECTION_OBJ_SCALE_Z__" + this->GetName(), this->scaleVisual));

  this->scaleXVisual->Load();
  this->scaleYVisual->Load();
  this->scaleZVisual->Load();

  this->InsertMesh("unit_box");

  Ogre::MovableObject *scaleShaftXObj =
      (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_SCALE_SHAFT_X__" + this->GetName(), "axis_shaft"));
  Ogre::MovableObject *scaleHeadXObj =
      (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_SCALE_HEAD_X__" + this->GetName(), "unit_box"));
  Ogre::SceneNode *scaleShaftXNode =
      this->scaleXVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ__SCALE_SHAFT_NODE_X__"  + this->GetName());
  Ogre::SceneNode *scaleHeadXNode =
      this->scaleXVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ__SCALE_HEAD_NODE_X__"  + this->GetName());
  scaleShaftXNode->attachObject(scaleShaftXObj);
  scaleShaftXNode->setScale(0.5, 0.5, 1.0);
  scaleShaftXNode->setPosition(0, 0, 0.1);
  scaleHeadXNode->attachObject(scaleHeadXObj);
  scaleHeadXNode->setScale(0.02, 0.02, 0.02);
  scaleHeadXNode->setPosition(0, 0, 0.21);
  scaleShaftXObj->setUserAny(Ogre::Any(std::string("scale_x")));
  scaleHeadXObj->setUserAny(Ogre::Any(std::string("scale_x")));
  scaleShaftXObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
  scaleHeadXObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);

  Ogre::MovableObject *scaleShaftYObj =
      (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_SCALE_SHAFT_Y__" + this->GetName(), "axis_shaft"));
  Ogre::MovableObject *scaleHeadYObj =
      (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_SCALE_HEAD_Y__" + this->GetName(), "unit_box"));
  Ogre::SceneNode *scaleShaftYNode =
      this->scaleYVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ_SCALE_SHAFT_NODE_Y__"  + this->GetName());
  Ogre::SceneNode *scaleHeadYNode =
      this->scaleYVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ_SCALE_HEAD_NODE_Y__"  + this->GetName());
  scaleShaftYNode->attachObject(scaleShaftYObj);
  scaleShaftYNode->setScale(0.5, 0.5, 1.0);
  scaleShaftYNode->setPosition(0, 0, 0.1);
  scaleHeadYNode->attachObject(scaleHeadYObj);
  scaleHeadYNode->setScale(0.02, 0.02, 0.02);
  scaleHeadYNode->setPosition(0, 0, 0.21);
  scaleShaftYObj->setUserAny(Ogre::Any(std::string("scale_y")));
  scaleHeadYObj->setUserAny(Ogre::Any(std::string("scale_y")));
  scaleShaftYObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
  scaleHeadYObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);

  Ogre::MovableObject *scaleShaftZObj =
      (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_SCALE_SHAFT_Z__" + this->GetName(), "axis_shaft"));
  Ogre::MovableObject *scaleHeadZObj =
      (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_SCALE_HEAD_Z__" + this->GetName(), "unit_box"));
  Ogre::SceneNode *scaleShaftZNode =
      this->scaleZVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ_SCALE_SHAFT_NODE_Z__"  + this->GetName());
  Ogre::SceneNode *scaleHeadZNode =
      this->scaleZVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ_SCALE_HEAD_NODE_Z__"  + this->GetName());
  scaleShaftZNode->attachObject(scaleShaftZObj);
  scaleShaftZNode->setScale(0.5, 0.5, 1.0);
  scaleShaftZNode->setPosition(0, 0, 0.1);
  scaleHeadZNode->attachObject(scaleHeadZObj);
  scaleHeadZNode->setScale(0.02, 0.02, 0.02);
  scaleHeadZNode->setPosition(0, 0, 0.21);
  scaleShaftZObj->setUserAny(Ogre::Any(std::string("scale_z")));
  scaleHeadZObj->setUserAny(Ogre::Any(std::string("scale_z")));
  scaleShaftZObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
  scaleHeadZObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);

  this->scaleXVisual->SetRotation(
      math::Quaternion(math::Vector3(0, 1, 0), GZ_DTOR(90)));
  this->scaleYVisual->SetRotation(
      math::Quaternion(math::Vector3(1, 0, 0), GZ_DTOR(-90)));

  this->scaleXVisual->SetMaterial(this->xAxisMatOverlay);
  this->scaleYVisual->SetMaterial(this->yAxisMatOverlay);
  this->scaleZVisual->SetMaterial(this->zAxisMatOverlay);

  this->scaleVisual->SetScale(math::Vector3(5.0, 5.0, 5.0));

  this->scaleXVisual->SetVisibilityFlags(
      GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE);
  this->scaleYVisual->SetVisibilityFlags(
      GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE);
  this->scaleZVisual->SetVisibilityFlags(
      GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE);

  // Add to scene so they are selectable by the mouse
  this->scene->AddVisual(this->scaleXVisual);
  this->scene->AddVisual(this->scaleYVisual);
  this->scene->AddVisual(this->scaleZVisual);
}
