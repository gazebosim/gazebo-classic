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

#include "gazebo/common/MeshManager.hh"

#include "gazebo/gui/GuiIface.hh"

#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/SelectionObjPrivate.hh"
#include "gazebo/rendering/SelectionObj.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
SelectionObj::SelectionObj(const std::string &_name, VisualPtr _vis)
  : Visual(*new SelectionObjPrivate, _name, _vis, false)
{
  SelectionObjPrivate *dPtr =
      reinterpret_cast<SelectionObjPrivate *>(this->dataPtr);

  dPtr->state = SELECTION_NONE;
  dPtr->mode = SELECTION_NONE;

  dPtr->maxScale = 2.5;
  dPtr->minScale = 0.5;

  dPtr->xAxisMatOverlay = "Gazebo/RedTransparentOverlay";
  dPtr->yAxisMatOverlay = "Gazebo/GreenTransparentOverlay";
  dPtr->zAxisMatOverlay = "Gazebo/BlueTransparentOverlay";

  dPtr->xAxisMat = "Gazebo/RedTransparent";
  dPtr->yAxisMat = "Gazebo/GreenTransparent";
  dPtr->zAxisMat = "Gazebo/BlueTransparent";
}

/////////////////////////////////////////////////
SelectionObj::~SelectionObj()
{
  SelectionObjPrivate *dPtr =
      reinterpret_cast<SelectionObjPrivate *>(this->dataPtr);
  dPtr->parent.reset();
}

/////////////////////////////////////////////////
void SelectionObj::Load()
{
  Visual::Load();

  this->CreateRotateVisual();
  this->CreateTranslateVisual();
  this->CreateScaleVisual();

  this->SetVisibilityFlags(GZ_VISIBILITY_GUI);

  this->SetHandleVisible(TRANS, false);
  this->SetHandleVisible(ROT, false);
  this->SetHandleVisible(SCALE, false);

  this->GetSceneNode()->setInheritScale(false);
}

/////////////////////////////////////////////////
void SelectionObj::Attach(rendering::VisualPtr _vis)
{
  SelectionObjPrivate *dPtr =
      reinterpret_cast<SelectionObjPrivate *>(this->dataPtr);

  if (dPtr->parent)
  {
    if (dPtr->parent == _vis)
      return;
    dPtr->parent->DetachVisual(shared_from_this());
  }

  dPtr->parent = _vis;
  dPtr->parent->AttachVisual(shared_from_this());
  this->SetPosition(math::Vector3(0, 0, 0));

  this->UpdateSize();
}

/////////////////////////////////////////////////
void SelectionObj::UpdateSize()
{
  SelectionObjPrivate *dPtr =
      reinterpret_cast<SelectionObjPrivate *>(this->dataPtr);

  VisualPtr vis = dPtr->parent;

  // don't include the selection obj itself when calculating the size.
  this->Detach();
  math::Vector3 bboxSize = vis->GetBoundingBox().GetSize()
      * vis->GetScale();
  dPtr->parent = vis;
  dPtr->parent->AttachVisual(shared_from_this());

  double max = std::max(std::max(bboxSize.x, bboxSize.y), bboxSize.z);

  max = std::min(std::max(dPtr->minScale, max), dPtr->maxScale);

  // Handle special case for rotation visuals. Only set the visuals to be
  // overlays for big objects.
  if (math::equal(max, dPtr->maxScale))
  {
    this->SetHandleMaterial(ROT_X, dPtr->xAxisMatOverlay, false);
    this->SetHandleMaterial(ROT_Y, dPtr->yAxisMatOverlay, false);
    this->SetHandleMaterial(ROT_Z, dPtr->zAxisMatOverlay, false);
  }
  else
  {
    this->SetHandleMaterial(ROT_X, dPtr->xAxisMat, false);
    this->SetHandleMaterial(ROT_Y, dPtr->yAxisMat, false);
    this->SetHandleMaterial(ROT_Z, dPtr->zAxisMat, false);
  }
  this->SetScale(math::Vector3(max, max, max));
}

/////////////////////////////////////////////////
void SelectionObj::Detach()
{
  SelectionObjPrivate *dPtr =
      reinterpret_cast<SelectionObjPrivate *>(this->dataPtr);

  if (dPtr->parent)
    dPtr->parent->DetachVisual(shared_from_this());
  dPtr->parent.reset();
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
  SelectionObjPrivate *dPtr =
      reinterpret_cast<SelectionObjPrivate *>(this->dataPtr);

  if (_mode == dPtr->mode)
    return;

  dPtr->mode = _mode;

  this->SetHandleVisible(TRANS, false);
  this->SetHandleVisible(ROT, false);
  this->SetHandleVisible(SCALE, false);

  this->SetHandleVisible(dPtr->mode, true);
}

/////////////////////////////////////////////////
SelectionObj::SelectionMode SelectionObj::GetMode()
{
  SelectionObjPrivate *dPtr =
      reinterpret_cast<SelectionObjPrivate *>(this->dataPtr);

  return dPtr->mode;
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
  SelectionObjPrivate *dPtr =
      reinterpret_cast<SelectionObjPrivate *>(this->dataPtr);

  if (dPtr->state == _state)
    return;

  dPtr->state = _state;

  if (dPtr->selectedVis)
  {
    Ogre::MaterialPtr mat =
      Ogre::MaterialManager::getSingleton().getByName(
      dPtr->selectedVis->GetMaterialName());
    mat->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setAlphaOperation(
      Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, 0.5);
    dPtr->selectedVis.reset();
  }

  if (dPtr->state == TRANS_X)
    dPtr->selectedVis = dPtr->transXVisual;
  else if (dPtr->state == TRANS_Y)
    dPtr->selectedVis = dPtr->transYVisual;
  else if (dPtr->state == TRANS_Z)
    dPtr->selectedVis = dPtr->transZVisual;
  else if (dPtr->state == ROT_X)
    dPtr->selectedVis = dPtr->rotXVisual;
  else if (dPtr->state == ROT_Y)
    dPtr->selectedVis = dPtr->rotYVisual;
  else if (dPtr->state == ROT_Z)
    dPtr->selectedVis = dPtr->rotZVisual;
  else if (dPtr->state == SCALE_X)
    dPtr->selectedVis = dPtr->scaleXVisual;
  else if (dPtr->state == SCALE_Y)
    dPtr->selectedVis = dPtr->scaleYVisual;
  else if (dPtr->state == SCALE_Z)
    dPtr->selectedVis = dPtr->scaleZVisual;

  if (dPtr->selectedVis)
  {
    Ogre::MaterialPtr mat =
      Ogre::MaterialManager::getSingleton().getByName(
      dPtr->selectedVis->GetMaterialName());
    mat->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setAlphaOperation(
      Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, 0.7);
  }
}

/////////////////////////////////////////////////
void SelectionObj::SetGlobal(bool _global)
{
  SelectionObjPrivate *dPtr =
      reinterpret_cast<SelectionObjPrivate *>(this->dataPtr);

  dPtr->transVisual->GetSceneNode()->setInheritOrientation(!_global);
  dPtr->rotVisual->GetSceneNode()->setInheritOrientation(!_global);
}

/////////////////////////////////////////////////
SelectionObj::SelectionMode SelectionObj::GetState()
{
  SelectionObjPrivate *dPtr =
      reinterpret_cast<SelectionObjPrivate *>(this->dataPtr);

  return dPtr->state;
}

/////////////////////////////////////////////////
void SelectionObj::CreateTranslateVisual()
{
  SelectionObjPrivate *dPtr =
      reinterpret_cast<SelectionObjPrivate *>(this->dataPtr);

  // Translation mainipulation tool
  dPtr->transVisual.reset(new rendering::Visual(
      this->GetName() + "__SELECTION_OBJ_TRANS__",
      shared_from_this()));

  dPtr->transXVisual.reset(
      new rendering::Visual(
      "__SELECTION_OBJ_TRANS_X__" + this->GetName(), dPtr->transVisual));
  dPtr->transYVisual.reset(
      new rendering::Visual(
      "__SELECTION_OBJ_TRANS_Y__" + this->GetName(), dPtr->transVisual));
  dPtr->transZVisual.reset(
      new rendering::Visual(
      "__SELECTION_OBJ_TRANS_Z__" + this->GetName(), dPtr->transVisual));

  dPtr->transXVisual->Load();
  dPtr->transYVisual->Load();
  dPtr->transZVisual->Load();

  this->InsertMesh("axis_shaft");
  this->InsertMesh("axis_head");

  Ogre::MovableObject *shaftXObj =
      (Ogre::MovableObject*)(dPtr->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_TRANS_SHAFT_X__" + this->GetName(), "axis_shaft"));
  Ogre::MovableObject *headXObj =
      (Ogre::MovableObject*)(dPtr->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_TRANS_HEAD_X__" + this->GetName(), "axis_head"));
  Ogre::SceneNode *transShaftXNode =
      dPtr->transXVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ__TRANS_SHAFT_NODE_X__"  + this->GetName());
  Ogre::SceneNode *transHeadXNode =
      dPtr->transXVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ__TRANS_HEAD_NODE_X__"  + this->GetName());
  transShaftXNode->attachObject(shaftXObj);
  transShaftXNode->setScale(0.5, 0.5, 1.0);
  transShaftXNode->setPosition(0, 0, 0.1);
  transHeadXNode->attachObject(headXObj);
  transHeadXNode->setScale(0.5, 0.5, 0.5);
  transHeadXNode->setPosition(0, 0, 0.22);
  shaftXObj->getUserObjectBindings().setUserAny(
      Ogre::Any(std::string("trans_x")));
  headXObj->getUserObjectBindings().setUserAny(
      Ogre::Any(std::string("trans_x")));
  shaftXObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
  headXObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);

  Ogre::MovableObject *shaftYObj =
      (Ogre::MovableObject*)(dPtr->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_TRANS_SHAFT_Y__" + this->GetName(), "axis_shaft"));
  Ogre::MovableObject *headYObj =
      (Ogre::MovableObject*)(dPtr->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_TRANS_HEAD_Y__" + this->GetName(), "axis_head"));
  Ogre::SceneNode *transShaftYNode =
      dPtr->transYVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ_TRANS_SHAFT_NODE_Y__"  + this->GetName());
  Ogre::SceneNode *transHeadYNode =
      dPtr->transYVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ_TRANS_HEAD_NODE_Y__"  + this->GetName());
  transShaftYNode->attachObject(shaftYObj);
  transShaftYNode->setScale(0.5, 0.5, 1.0);
  transShaftYNode->setPosition(0, 0, 0.1);
  transHeadYNode->attachObject(headYObj);
  transHeadYNode->setScale(0.5, 0.5, 0.5);
  transHeadYNode->setPosition(0, 0, 0.22);
  shaftYObj->getUserObjectBindings().setUserAny(
      Ogre::Any(std::string("trans_y")));
  headYObj->getUserObjectBindings().setUserAny(
      Ogre::Any(std::string("trans_y")));
  shaftYObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
  headYObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);

  Ogre::MovableObject *shaftZObj =
      (Ogre::MovableObject*)(dPtr->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_TRANS_SHAFT_Z__" + this->GetName(), "axis_shaft"));
  Ogre::MovableObject *headZObj =
      (Ogre::MovableObject*)(dPtr->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_TRANS_HEAD_Z__" + this->GetName(), "axis_head"));
  Ogre::SceneNode *transShaftZNode =
      dPtr->transZVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ_TRANS_SHAFT_NODE_Z__"  + this->GetName());
  Ogre::SceneNode *transHeadZNode =
      dPtr->transZVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ_TRANS_HEAD_NODE_Z__"  + this->GetName());
  transShaftZNode->attachObject(shaftZObj);
  transShaftZNode->setScale(0.5, 0.5, 1.0);
  transShaftZNode->setPosition(0, 0, 0.1);
  transHeadZNode->attachObject(headZObj);
  transHeadZNode->setScale(0.5, 0.5, 0.5);
  transHeadZNode->setPosition(0, 0, 0.22);
  shaftZObj->getUserObjectBindings().setUserAny(
      Ogre::Any(std::string("trans_z")));
  headZObj->getUserObjectBindings().setUserAny(
      Ogre::Any(std::string("trans_z")));
  shaftZObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
  headZObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);

  dPtr->transXVisual->SetRotation(
      math::Quaternion(math::Vector3(0, 1, 0), GZ_DTOR(90)));
  dPtr->transYVisual->SetRotation(
      math::Quaternion(math::Vector3(1, 0, 0), GZ_DTOR(-90)));

  this->SetHandleMaterial(TRANS_X, dPtr->xAxisMatOverlay);
  this->SetHandleMaterial(TRANS_Y, dPtr->yAxisMatOverlay);
  this->SetHandleMaterial(TRANS_Z, dPtr->zAxisMatOverlay);

  dPtr->transVisual->SetScale(math::Vector3(5.0, 5.0, 5.0));

  dPtr->transXVisual->SetVisibilityFlags(
      GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE);
  dPtr->transYVisual->SetVisibilityFlags(
      GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE);
  dPtr->transZVisual->SetVisibilityFlags(
      GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE);

  // Add to scene so they are selectable by the mouse
  dPtr->scene->AddVisual(dPtr->transXVisual);
  dPtr->scene->AddVisual(dPtr->transYVisual);
  dPtr->scene->AddVisual(dPtr->transZVisual);
}

/////////////////////////////////////////////////
void SelectionObj::CreateRotateVisual()
{
  SelectionObjPrivate *dPtr =
      reinterpret_cast<SelectionObjPrivate *>(this->dataPtr);

  // Rotation mainipulation tool
  dPtr->rotVisual.reset(new rendering::Visual(
      this->GetName() + "__SELECTION_OBJ_ROT__",
      shared_from_this()));

  dPtr->rotXVisual.reset(
      new rendering::Visual(
      this->GetName() + "__SELECTION_OBJ_ROT_X__", dPtr->rotVisual));
  dPtr->rotYVisual.reset(
      new rendering::Visual(
      this->GetName() + "__SELECTION_OBJ_ROT_Y__", dPtr->rotVisual));
  dPtr->rotZVisual.reset(
      new rendering::Visual(
      this->GetName() + "__SELECTION_OBJ_ROT_Z__", dPtr->rotVisual));

  dPtr->rotVisual->InsertMesh("selection_tube");

  Ogre::MovableObject *rotXObj =
      (Ogre::MovableObject*)(dPtr->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_ROT_X__" + this->GetName(), "selection_tube"));
  Ogre::SceneNode *xNode =
      dPtr->rotXVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ__ROT_NODE_X__"  + this->GetName());
  xNode->attachObject(rotXObj);
  rotXObj->getUserObjectBindings().setUserAny(
      Ogre::Any(std::string("rot_x")));
  rotXObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);

  Ogre::MovableObject *rotYObj =
      (Ogre::MovableObject*)(dPtr->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_ROT_Y__" + this->GetName(), "selection_tube"));
  Ogre::SceneNode *yNode =
      dPtr->rotYVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ__ROT_NODE_Y__"  + this->GetName());
  yNode->attachObject(rotYObj);
  rotYObj->getUserObjectBindings().setUserAny(
      Ogre::Any(std::string("rot_y")));
  rotYObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);

  Ogre::MovableObject *rotZObj =
      (Ogre::MovableObject*)(dPtr->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_ROT_Z__" + this->GetName(), "selection_tube"));
  Ogre::SceneNode *zNode =
      dPtr->rotZVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ__ROT_NODE_Z__"  + this->GetName());
  zNode->attachObject(rotZObj);
  rotZObj->getUserObjectBindings().setUserAny(
      Ogre::Any(std::string("rot_z")));
  rotZObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);

  dPtr->rotXVisual->Load();
  dPtr->rotYVisual->Load();
  dPtr->rotZVisual->Load();

  dPtr->rotXVisual->SetRotation(
      math::Quaternion(math::Vector3(0, 1, 0), GZ_DTOR(90)));
  dPtr->rotYVisual->SetRotation(
      math::Quaternion(math::Vector3(1, 0, 0), GZ_DTOR(-90)));

  // By default the visuals are not overlays like translation or scale visuals.
  // This is so that the rings does not block the object it's attached too,
  // and also gives with better depth perception.
  this->SetHandleMaterial(ROT_X, dPtr->xAxisMat);
  this->SetHandleMaterial(ROT_Y, dPtr->yAxisMat);
  this->SetHandleMaterial(ROT_Z, dPtr->zAxisMat);

  dPtr->rotVisual->SetScale(math::Vector3(1.0, 1.0, 1.0));

  dPtr->rotXVisual->SetVisibilityFlags(
      GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE);
  dPtr->rotYVisual->SetVisibilityFlags(
      GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE);
  dPtr->rotZVisual->SetVisibilityFlags(
      GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE);

  // Add to scene so they are selectable by the mouse
  dPtr->scene->AddVisual(dPtr->rotXVisual);
  dPtr->scene->AddVisual(dPtr->rotYVisual);
  dPtr->scene->AddVisual(dPtr->rotZVisual);
}

/////////////////////////////////////////////////
void SelectionObj::CreateScaleVisual()
{
  SelectionObjPrivate *dPtr =
      reinterpret_cast<SelectionObjPrivate *>(this->dataPtr);

  // Scale mainipulation tool
  dPtr->scaleVisual.reset(new rendering::Visual(
      this->GetName() + "__SELECTION_OBJ_SCALE__",
      shared_from_this()));

  dPtr->scaleXVisual.reset(
      new rendering::Visual(
      "__SELECTION_OBJ_SCALE_X__" + this->GetName(), dPtr->scaleVisual));
  dPtr->scaleYVisual.reset(
      new rendering::Visual(
      "__SELECTION_OBJ_SCALE_Y__" + this->GetName(), dPtr->scaleVisual));
  dPtr->scaleZVisual.reset(
      new rendering::Visual(
      "__SELECTION_OBJ_SCALE_Z__" + this->GetName(), dPtr->scaleVisual));

  dPtr->scaleXVisual->Load();
  dPtr->scaleYVisual->Load();
  dPtr->scaleZVisual->Load();

  this->InsertMesh("unit_box");

  Ogre::MovableObject *scaleShaftXObj =
      (Ogre::MovableObject*)(dPtr->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_SCALE_SHAFT_X__" + this->GetName(), "axis_shaft"));
  Ogre::MovableObject *scaleHeadXObj =
      (Ogre::MovableObject*)(dPtr->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_SCALE_HEAD_X__" + this->GetName(), "unit_box"));
  Ogre::SceneNode *scaleShaftXNode =
      dPtr->scaleXVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ__SCALE_SHAFT_NODE_X__"  + this->GetName());
  Ogre::SceneNode *scaleHeadXNode =
      dPtr->scaleXVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ__SCALE_HEAD_NODE_X__"  + this->GetName());
  scaleShaftXNode->attachObject(scaleShaftXObj);
  scaleShaftXNode->setScale(0.5, 0.5, 1.0);
  scaleShaftXNode->setPosition(0, 0, 0.1);
  scaleHeadXNode->attachObject(scaleHeadXObj);
  scaleHeadXNode->setScale(0.02, 0.02, 0.02);
  scaleHeadXNode->setPosition(0, 0, 0.21);
  scaleShaftXObj->getUserObjectBindings().setUserAny(
      Ogre::Any(std::string("scale_x")));
  scaleHeadXObj->getUserObjectBindings().setUserAny(
      Ogre::Any(std::string("scale_x")));
  scaleShaftXObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
  scaleHeadXObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);

  Ogre::MovableObject *scaleShaftYObj =
      (Ogre::MovableObject*)(dPtr->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_SCALE_SHAFT_Y__" + this->GetName(), "axis_shaft"));
  Ogre::MovableObject *scaleHeadYObj =
      (Ogre::MovableObject*)(dPtr->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_SCALE_HEAD_Y__" + this->GetName(), "unit_box"));
  Ogre::SceneNode *scaleShaftYNode =
      dPtr->scaleYVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ_SCALE_SHAFT_NODE_Y__"  + this->GetName());
  Ogre::SceneNode *scaleHeadYNode =
      dPtr->scaleYVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ_SCALE_HEAD_NODE_Y__"  + this->GetName());
  scaleShaftYNode->attachObject(scaleShaftYObj);
  scaleShaftYNode->setScale(0.5, 0.5, 1.0);
  scaleShaftYNode->setPosition(0, 0, 0.1);
  scaleHeadYNode->attachObject(scaleHeadYObj);
  scaleHeadYNode->setScale(0.02, 0.02, 0.02);
  scaleHeadYNode->setPosition(0, 0, 0.21);
  scaleShaftYObj->getUserObjectBindings().setUserAny(
      Ogre::Any(std::string("scale_y")));
  scaleHeadYObj->getUserObjectBindings().setUserAny(
      Ogre::Any(std::string("scale_y")));
  scaleShaftYObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
  scaleHeadYObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);

  Ogre::MovableObject *scaleShaftZObj =
      (Ogre::MovableObject*)(dPtr->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_SCALE_SHAFT_Z__" + this->GetName(), "axis_shaft"));
  Ogre::MovableObject *scaleHeadZObj =
      (Ogre::MovableObject*)(dPtr->scene->GetManager()->createEntity(
      "__SELECTION_OBJ_SCALE_HEAD_Z__" + this->GetName(), "unit_box"));
  Ogre::SceneNode *scaleShaftZNode =
      dPtr->scaleZVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ_SCALE_SHAFT_NODE_Z__"  + this->GetName());
  Ogre::SceneNode *scaleHeadZNode =
      dPtr->scaleZVisual->GetSceneNode()->createChildSceneNode(
      "__SELECTION_OBJ_SCALE_HEAD_NODE_Z__"  + this->GetName());
  scaleShaftZNode->attachObject(scaleShaftZObj);
  scaleShaftZNode->setScale(0.5, 0.5, 1.0);
  scaleShaftZNode->setPosition(0, 0, 0.1);
  scaleHeadZNode->attachObject(scaleHeadZObj);
  scaleHeadZNode->setScale(0.02, 0.02, 0.02);
  scaleHeadZNode->setPosition(0, 0, 0.21);
  scaleShaftZObj->getUserObjectBindings().setUserAny(
      Ogre::Any(std::string("scale_z")));
  scaleHeadZObj->getUserObjectBindings().setUserAny(
      Ogre::Any(std::string("scale_z")));
  scaleShaftZObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
  scaleHeadZObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);

  dPtr->scaleXVisual->SetRotation(
      math::Quaternion(math::Vector3(0, 1, 0), GZ_DTOR(90)));
  dPtr->scaleYVisual->SetRotation(
      math::Quaternion(math::Vector3(1, 0, 0), GZ_DTOR(-90)));

  this->SetHandleMaterial(SCALE_X, dPtr->xAxisMatOverlay);
  this->SetHandleMaterial(SCALE_Y, dPtr->yAxisMatOverlay);
  this->SetHandleMaterial(SCALE_Z, dPtr->zAxisMatOverlay);

  dPtr->scaleVisual->SetScale(math::Vector3(5.0, 5.0, 5.0));

  dPtr->scaleXVisual->SetVisibilityFlags(
      GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE);
  dPtr->scaleYVisual->SetVisibilityFlags(
      GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE);
  dPtr->scaleZVisual->SetVisibilityFlags(
      GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE);

  // Add to scene so they are selectable by the mouse
  dPtr->scene->AddVisual(dPtr->scaleXVisual);
  dPtr->scene->AddVisual(dPtr->scaleYVisual);
  dPtr->scene->AddVisual(dPtr->scaleZVisual);
}

/////////////////////////////////////////////////
void SelectionObj::SetHandleVisible(SelectionMode _mode, bool _visible)
{
  SelectionObjPrivate *dPtr =
      reinterpret_cast<SelectionObjPrivate *>(this->dataPtr);

  if (_mode == TRANS || _mode == TRANS_X)
    dPtr->transXVisual->SetVisible(_visible);
  if (_mode == TRANS || _mode == TRANS_Y)
    dPtr->transYVisual->SetVisible(_visible);
  if (_mode == TRANS || _mode == TRANS_Z)
    dPtr->transZVisual->SetVisible(_visible);
  if (_mode == ROT || _mode == ROT_X)
    dPtr->rotXVisual->SetVisible(_visible);
  if (_mode == ROT || _mode == ROT_Y)
    dPtr->rotYVisual->SetVisible(_visible);
  if (_mode == ROT || _mode == ROT_Z)
    dPtr->rotZVisual->SetVisible(_visible);
  if (_mode == SCALE || _mode == SCALE_X)
    dPtr->scaleXVisual->SetVisible(_visible);
  if (_mode == SCALE || _mode == SCALE_Y)
    dPtr->scaleYVisual->SetVisible(_visible);
  if (_mode == SCALE || _mode == SCALE_Z)
    dPtr->scaleZVisual->SetVisible(_visible);
}

/////////////////////////////////////////////////
bool SelectionObj::GetHandleVisible(SelectionMode _mode) const
{
  SelectionObjPrivate *dPtr =
      reinterpret_cast<SelectionObjPrivate *>(this->dataPtr);

  if (_mode == TRANS || _mode == TRANS_X)
    return dPtr->transXVisual->GetVisible();
  if (_mode == TRANS_Y)
    return dPtr->transYVisual->GetVisible();
  if (_mode == TRANS_Z)
    return dPtr->transZVisual->GetVisible();
  if (_mode == ROT || _mode == ROT_X)
    return dPtr->rotXVisual->GetVisible();
  if (_mode == ROT_Y)
    return dPtr->rotYVisual->GetVisible();
  if (_mode == ROT_Z)
    return dPtr->rotZVisual->GetVisible();
  if (_mode == SCALE || _mode == SCALE_X)
    return dPtr->scaleXVisual->GetVisible();
  if (_mode == SCALE_Y)
    return dPtr->scaleYVisual->GetVisible();
  if (_mode == SCALE_Z)
    return dPtr->scaleZVisual->GetVisible();

  return false;
}

/////////////////////////////////////////////////
void SelectionObj::SetHandleMaterial(SelectionMode _mode, const std::string
    &_material, bool _unique)
{
  SelectionObjPrivate *dPtr =
      reinterpret_cast<SelectionObjPrivate *>(this->dataPtr);

  if (_mode == TRANS || _mode == TRANS_X)
    dPtr->transXVisual->SetMaterial(_material, _unique);
  if (_mode == TRANS || _mode == TRANS_Y)
    dPtr->transYVisual->SetMaterial(_material, _unique);
  if (_mode == TRANS || _mode == TRANS_Z)
    dPtr->transZVisual->SetMaterial(_material, _unique);
  if (_mode == ROT || _mode == ROT_X)
    dPtr->rotXVisual->SetMaterial(_material, _unique);
  if (_mode == ROT || _mode == ROT_Y)
    dPtr->rotYVisual->SetMaterial(_material, _unique);
  if (_mode == ROT || _mode == ROT_Z)
    dPtr->rotZVisual->SetMaterial(_material, _unique);
  if (_mode == SCALE || _mode == SCALE_X)
    dPtr->scaleXVisual->SetMaterial(_material, _unique);
  if (_mode == SCALE || _mode == SCALE_Y)
    dPtr->scaleYVisual->SetMaterial(_material, _unique);
  if (_mode == SCALE || _mode == SCALE_Z)
    dPtr->scaleZVisual->SetMaterial(_material, _unique);
}
