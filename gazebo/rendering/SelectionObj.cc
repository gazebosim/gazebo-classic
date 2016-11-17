/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include "gazebo/common/MeshManager.hh"


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

  dPtr->type = VT_GUI;

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

  this->SetVisibilityFlags(GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE);

  this->SetHandleVisible(TRANS, false);
  this->SetHandleVisible(ROT, false);
  this->SetHandleVisible(SCALE, false);

  this->GetSceneNode()->setInheritScale(false);
  this->SetInheritTransparency(false);
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
  this->SetPosition(ignition::math::Vector3d::Zero);

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
  this->SetScale(ignition::math::Vector3d(max, max, max));
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
    dPtr->selectedVis->SetTransparency(0.5);
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
    dPtr->selectedVis->SetTransparency(0.3);
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
      shared_from_this(), false));
  dPtr->transVisual->Load();

  dPtr->transXVisual.reset(
      new rendering::Visual(
      "__SELECTION_OBJ_TRANS_X__" + this->GetName(), dPtr->transVisual, false));
  dPtr->transYVisual.reset(
      new rendering::Visual(
      "__SELECTION_OBJ_TRANS_Y__" + this->GetName(), dPtr->transVisual, false));
  dPtr->transZVisual.reset(
      new rendering::Visual(
      "__SELECTION_OBJ_TRANS_Z__" + this->GetName(), dPtr->transVisual, false));

  dPtr->transXVisual->Load();
  dPtr->transYVisual->Load();
  dPtr->transZVisual->Load();

  this->InsertMesh("axis_shaft");
  this->InsertMesh("axis_head");

  VisualPtr transShaftXVis(
      new Visual("__SELECTION_OBJ__TRANS_SHAFT_NODE_X__"  + this->GetName(),
      dPtr->transXVisual, false));
  transShaftXVis->Load();
  transShaftXVis->AttachMesh("axis_shaft");
  transShaftXVis->SetScale(ignition::math::Vector3d(0.5, 0.5, 1.0));
  transShaftXVis->SetPosition(ignition::math::Vector3d(0, 0, 0.1));
  Ogre::MovableObject *shaftXObj =
      transShaftXVis->GetSceneNode()->getAttachedObject(0);
  shaftXObj->getUserObjectBindings().setUserAny(
        Ogre::Any(std::string("trans_x")));
  shaftXObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);

  VisualPtr transHeadXVis(
      new Visual("__SELECTION_OBJ__TRANS_HEAD_NODE_X__"  + this->GetName(),
      dPtr->transXVisual, false));
  transHeadXVis->Load();
  transHeadXVis->AttachMesh("axis_head");
  transHeadXVis->SetScale(ignition::math::Vector3d(0.5, 0.5, 0.5));
  transHeadXVis->SetPosition(ignition::math::Vector3d(0, 0, 0.22));
  Ogre::MovableObject *headXObj =
      transHeadXVis->GetSceneNode()->getAttachedObject(0);
  headXObj->getUserObjectBindings().setUserAny(
        Ogre::Any(std::string("trans_x")));
  headXObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);

  VisualPtr transShaftYVis(
      new Visual("__SELECTION_OBJ__TRANS_SHAFT_NODE_Y__"  + this->GetName(),
      dPtr->transYVisual, false));
  transShaftYVis->Load();
  transShaftYVis->AttachMesh("axis_shaft");
  transShaftYVis->SetScale(ignition::math::Vector3d(0.5, 0.5, 1.0));
  transShaftYVis->SetPosition(ignition::math::Vector3d(0, 0, 0.1));
  Ogre::MovableObject *shaftYObj =
      transShaftYVis->GetSceneNode()->getAttachedObject(0);
  shaftYObj->getUserObjectBindings().setUserAny(
      Ogre::Any(std::string("trans_y")));
  shaftYObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);

  VisualPtr transHeadYVis(
      new Visual("__SELECTION_OBJ__TRANS_HEAD_NODE_Y__"  + this->GetName(),
      dPtr->transYVisual, false));
  transHeadYVis->Load();
  transHeadYVis->AttachMesh("axis_head");
  transHeadYVis->SetScale(ignition::math::Vector3d(0.5, 0.5, 0.5));
  transHeadYVis->SetPosition(ignition::math::Vector3d(0, 0, 0.22));
  Ogre::MovableObject *headYObj =
      transHeadYVis->GetSceneNode()->getAttachedObject(0);
  headYObj->getUserObjectBindings().setUserAny(
      Ogre::Any(std::string("trans_y")));
  headYObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);

  VisualPtr transShaftZVis(
      new Visual("__SELECTION_OBJ__TRANS_SHAFT_NODE_Z__"  + this->GetName(),
      dPtr->transZVisual, false));
  transShaftZVis->Load();
  transShaftZVis->AttachMesh("axis_shaft");
  transShaftZVis->SetScale(ignition::math::Vector3d(0.5, 0.5, 1.0));
  transShaftZVis->SetPosition(ignition::math::Vector3d(0, 0, 0.1));
  Ogre::MovableObject *shaftZObj =
      transShaftZVis->GetSceneNode()->getAttachedObject(0);
  shaftZObj->getUserObjectBindings().setUserAny(
      Ogre::Any(std::string("trans_z")));
  shaftZObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);

  VisualPtr transHeadZVis(
      new Visual("__SELECTION_OBJ__TRANS_HEAD_NODE_Z__"  + this->GetName(),
      dPtr->transZVisual, false));
  transHeadZVis->Load();
  transHeadZVis->AttachMesh("axis_head");
  transHeadZVis->SetScale(ignition::math::Vector3d(0.5, 0.5, 0.5));
  transHeadZVis->SetPosition(ignition::math::Vector3d(0, 0, 0.22));
  Ogre::MovableObject *headZObj =
      transHeadZVis->GetSceneNode()->getAttachedObject(0);
  headZObj->getUserObjectBindings().setUserAny(
      Ogre::Any(std::string("trans_z")));
  headZObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);

  dPtr->transXVisual->SetRotation(
      math::Quaternion(math::Vector3(0, 1, 0), GZ_DTOR(90)));
  dPtr->transYVisual->SetRotation(
      math::Quaternion(math::Vector3(1, 0, 0), GZ_DTOR(-90)));

  this->SetHandleMaterial(TRANS_X, dPtr->xAxisMatOverlay);
  this->SetHandleMaterial(TRANS_Y, dPtr->yAxisMatOverlay);
  this->SetHandleMaterial(TRANS_Z, dPtr->zAxisMatOverlay);

  dPtr->transVisual->SetInheritTransparency(false);
  dPtr->transVisual->SetScale(ignition::math::Vector3d(5.0, 5.0, 5.0));

  // set transparency once and make sure they do not inherit
  // transparency anymore
  dPtr->transVisual->SetTransparency(0.5);
  dPtr->transXVisual->SetInheritTransparency(false);
  dPtr->transYVisual->SetInheritTransparency(false);
  dPtr->transZVisual->SetInheritTransparency(false);

  dPtr->transXVisual->SetVisibilityFlags(
      GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE);
  dPtr->transYVisual->SetVisibilityFlags(
      GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE);
  dPtr->transZVisual->SetVisibilityFlags(
      GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE);
}

/////////////////////////////////////////////////
void SelectionObj::CreateRotateVisual()
{
  SelectionObjPrivate *dPtr =
      reinterpret_cast<SelectionObjPrivate *>(this->dataPtr);

  // Rotation mainipulation tool
  dPtr->rotVisual.reset(new rendering::Visual(
      this->GetName() + "__SELECTION_OBJ_ROT__",
      shared_from_this(), false));
  dPtr->rotVisual->Load();

  dPtr->rotXVisual.reset(
      new rendering::Visual(
      this->GetName() + "__SELECTION_OBJ_ROT_X__", dPtr->rotVisual, false));
  dPtr->rotYVisual.reset(
      new rendering::Visual(
      this->GetName() + "__SELECTION_OBJ_ROT_Y__", dPtr->rotVisual, false));
  dPtr->rotZVisual.reset(
      new rendering::Visual(
      this->GetName() + "__SELECTION_OBJ_ROT_Z__", dPtr->rotVisual, false));

  dPtr->rotVisual->InsertMesh("selection_tube");

  dPtr->rotXVisual->Load();
  dPtr->rotYVisual->Load();
  dPtr->rotZVisual->Load();

  VisualPtr rotRingXVis(
      new Visual("__SELECTION_OBJ__ROT_NODE_X__"  + this->GetName(),
      dPtr->rotXVisual, false));
  rotRingXVis->Load();
  rotRingXVis->AttachMesh("selection_tube");
  Ogre::MovableObject *rotXObj =
      rotRingXVis->GetSceneNode()->getAttachedObject(0);
  rotXObj->getUserObjectBindings().setUserAny(
      Ogre::Any(std::string("rot_x")));
  rotXObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);

  VisualPtr rotRingYVis(
      new Visual("__SELECTION_OBJ__ROT_NODE_Y__"  + this->GetName(),
      dPtr->rotYVisual, false));
  rotRingYVis->Load();
  rotRingYVis->AttachMesh("selection_tube");
  Ogre::MovableObject *rotYObj =
      rotRingYVis->GetSceneNode()->getAttachedObject(0);
  rotYObj->getUserObjectBindings().setUserAny(
      Ogre::Any(std::string("rot_y")));
  rotYObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);

  VisualPtr rotRingZVis(
      new Visual("__SELECTION_OBJ__ROT_NODE_Z__"  + this->GetName(),
      dPtr->rotZVisual, false));
  rotRingZVis->Load();
  rotRingZVis->AttachMesh("selection_tube");
  Ogre::MovableObject *rotZObj =
      rotRingZVis->GetSceneNode()->getAttachedObject(0);
  rotZObj->getUserObjectBindings().setUserAny(
      Ogre::Any(std::string("rot_z")));
  rotZObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);

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

  dPtr->rotVisual->SetScale(ignition::math::Vector3d::One);

  // set transparency once and make sure they do not inherit
  // transparency anymore
  dPtr->rotVisual->SetTransparency(0.5);
  dPtr->rotXVisual->SetInheritTransparency(false);
  dPtr->rotYVisual->SetInheritTransparency(false);
  dPtr->rotZVisual->SetInheritTransparency(false);

  dPtr->rotXVisual->SetVisibilityFlags(
      GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE);
  dPtr->rotYVisual->SetVisibilityFlags(
      GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE);
  dPtr->rotZVisual->SetVisibilityFlags(
      GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE);
}

/////////////////////////////////////////////////
void SelectionObj::CreateScaleVisual()
{
  SelectionObjPrivate *dPtr =
      reinterpret_cast<SelectionObjPrivate *>(this->dataPtr);

  // Scale mainipulation tool
  dPtr->scaleVisual.reset(new rendering::Visual(
      this->GetName() + "__SELECTION_OBJ_SCALE__",
      shared_from_this(), false));
  dPtr->scaleVisual->Load();

  dPtr->scaleXVisual.reset(
      new rendering::Visual(
      "__SELECTION_OBJ_SCALE_X__" + this->GetName(), dPtr->scaleVisual, false));
  dPtr->scaleYVisual.reset(
      new rendering::Visual(
      "__SELECTION_OBJ_SCALE_Y__" + this->GetName(), dPtr->scaleVisual, false));
  dPtr->scaleZVisual.reset(
      new rendering::Visual(
      "__SELECTION_OBJ_SCALE_Z__" + this->GetName(), dPtr->scaleVisual, false));

  dPtr->scaleXVisual->Load();
  dPtr->scaleYVisual->Load();
  dPtr->scaleZVisual->Load();

  this->InsertMesh("unit_box");

  VisualPtr scaleShaftXVis(
      new Visual("__SELECTION_OBJ__SCALE_SHAFT_NODE_X__"  + this->GetName(),
      dPtr->scaleXVisual, false));
  scaleShaftXVis->Load();
  scaleShaftXVis->AttachMesh("axis_shaft");
  scaleShaftXVis->SetScale(ignition::math::Vector3d(0.5, 0.5, 1.0));
  scaleShaftXVis->SetPosition(ignition::math::Vector3d(0, 0, 0.1));
  Ogre::MovableObject *scaleXObj =
      scaleShaftXVis->GetSceneNode()->getAttachedObject(0);
  scaleXObj->getUserObjectBindings().setUserAny(
      Ogre::Any(std::string("scale_x")));
  scaleXObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);

  VisualPtr scaleHeadXVis(
      new Visual("__SELECTION_OBJ__SCALE_HEAD_NODE_X__"  + this->GetName(),
      dPtr->scaleXVisual, false));
  scaleHeadXVis->Load();
  scaleHeadXVis->AttachMesh("unit_box");
  scaleHeadXVis->SetScale(ignition::math::Vector3d(0.02, 0.02, 0.02));
  scaleHeadXVis->SetPosition(ignition::math::Vector3d(0, 0, 0.21));
  Ogre::MovableObject *headXObj =
      scaleHeadXVis->GetSceneNode()->getAttachedObject(0);
  headXObj->getUserObjectBindings().setUserAny(
      Ogre::Any(std::string("scale_x")));
  headXObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);

  VisualPtr scaleShaftYVis(
      new Visual("__SELECTION_OBJ__SCALE_SHAFT_NODE_Y__"  + this->GetName(),
      dPtr->scaleYVisual, false));
  scaleShaftYVis->Load();
  scaleShaftYVis->AttachMesh("axis_shaft");
  scaleShaftYVis->SetScale(ignition::math::Vector3d(0.5, 0.5, 1.0));
  scaleShaftYVis->SetPosition(ignition::math::Vector3d(0, 0, 0.1));
  Ogre::MovableObject *scaleYObj =
      scaleShaftYVis->GetSceneNode()->getAttachedObject(0);
  scaleYObj->getUserObjectBindings().setUserAny(
      Ogre::Any(std::string("scale_y")));
  scaleYObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);

  VisualPtr scaleHeadYVis(
      new Visual("__SELECTION_OBJ__SCALE_HEAD_NODE_Y__"  + this->GetName(),
      dPtr->scaleYVisual, false));
  scaleHeadYVis->Load();
  scaleHeadYVis->AttachMesh("unit_box");
  scaleHeadYVis->SetScale(ignition::math::Vector3d(0.02, 0.02, 0.02));
  scaleHeadYVis->SetPosition(ignition::math::Vector3d(0, 0, 0.21));
  Ogre::MovableObject *headYObj =
      scaleHeadYVis->GetSceneNode()->getAttachedObject(0);
  headYObj->getUserObjectBindings().setUserAny(
      Ogre::Any(std::string("scale_y")));
  headYObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);

  VisualPtr scaleShaftZVis(
      new Visual("__SELECTION_OBJ__SCALE_SHAFT_NODE_Z__"  + this->GetName(),
      dPtr->scaleZVisual, false));
  scaleShaftZVis->Load();
  scaleShaftZVis->AttachMesh("axis_shaft");
  scaleShaftZVis->SetScale(ignition::math::Vector3d(0.5, 0.5, 1.0));
  scaleShaftZVis->SetPosition(ignition::math::Vector3d(0, 0, 0.1));
  Ogre::MovableObject *scaleZObj =
      scaleShaftZVis->GetSceneNode()->getAttachedObject(0);
  scaleZObj->getUserObjectBindings().setUserAny(
      Ogre::Any(std::string("scale_z")));
  scaleZObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);

  VisualPtr scaleHeadZVis(
      new Visual("__SELECTION_OBJ__SCALE_HEAD_NODE_Z__"  + this->GetName(),
      dPtr->scaleZVisual, false));
  scaleHeadZVis->Load();
  scaleHeadZVis->AttachMesh("unit_box");
  scaleHeadZVis->SetScale(ignition::math::Vector3d(0.02, 0.02, 0.02));
  scaleHeadZVis->SetPosition(ignition::math::Vector3d(0, 0, 0.21));
  Ogre::MovableObject *headZObj =
      scaleHeadZVis->GetSceneNode()->getAttachedObject(0);
  headZObj->getUserObjectBindings().setUserAny(
      Ogre::Any(std::string("scale_z")));
  headZObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);

  dPtr->scaleXVisual->SetRotation(
      math::Quaternion(math::Vector3(0, 1, 0), GZ_DTOR(90)));
  dPtr->scaleYVisual->SetRotation(
      math::Quaternion(math::Vector3(1, 0, 0), GZ_DTOR(-90)));

  this->SetHandleMaterial(SCALE_X, dPtr->xAxisMatOverlay);
  this->SetHandleMaterial(SCALE_Y, dPtr->yAxisMatOverlay);
  this->SetHandleMaterial(SCALE_Z, dPtr->zAxisMatOverlay);

  dPtr->scaleVisual->SetScale(ignition::math::Vector3d(5.0, 5.0, 5.0));

  // set transparency once and make sure they do not inherit
  // transparency anymore
  dPtr->scaleVisual->SetTransparency(0.5);
  dPtr->scaleXVisual->SetInheritTransparency(false);
  dPtr->scaleYVisual->SetInheritTransparency(false);
  dPtr->scaleZVisual->SetInheritTransparency(false);

  dPtr->scaleXVisual->SetVisibilityFlags(
      GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE);
  dPtr->scaleYVisual->SetVisibilityFlags(
      GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE);
  dPtr->scaleZVisual->SetVisibilityFlags(
      GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE);
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
