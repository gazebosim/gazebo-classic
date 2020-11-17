/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include <ignition/common/Profiler.hh>
#include <ignition/math/Color.hh>
#include <ignition/math/Matrix4.hh>

#include "gazebo/common/MeshManager.hh"

#include "gazebo/rendering/Material.hh"
#include "gazebo/rendering/MovableText.hh"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/SelectionObj.hh"
#include "gazebo/rendering/ApplyWrenchVisualPrivate.hh"
#include "gazebo/rendering/ApplyWrenchVisual.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
ApplyWrenchVisual::ApplyWrenchVisual(const std::string &_name,
    VisualPtr _parentVis)
    : Visual(*new ApplyWrenchVisualPrivate, _name, _parentVis, false)
{
}

/////////////////////////////////////////////////
ApplyWrenchVisual::~ApplyWrenchVisual()
{
}

/////////////////////////////////////////////////
void ApplyWrenchVisual::Fini()
{
  ApplyWrenchVisualPrivate *dPtr =
      reinterpret_cast<ApplyWrenchVisualPrivate *>(this->dataPtr);

  // ogre will not be able to remove object of type MovableText so detach first.
  // objects are allocated on the stack so no need to delete
  if (dPtr->forceText.getParentNode())
    dPtr->forceText.detachFromParent();
  if (dPtr->torqueText.getParentNode())
    dPtr->torqueText.detachFromParent();

  Visual::Fini();
}

///////////////////////////////////////////////////
void ApplyWrenchVisual::Load()
{
  Visual::Load();

  ApplyWrenchVisualPrivate *dPtr =
      reinterpret_cast<ApplyWrenchVisualPrivate *>(this->dataPtr);

  if (!dPtr->scene)
  {
    gzerr << "Visual has no scene, not loading." << std::endl;
    return;
  }

  dPtr->selectedMaterial = "Gazebo/OrangeTransparentOverlay";
  dPtr->unselectedMaterial = "Gazebo/DarkOrangeTransparentOverlay";

  this->InsertMesh("axis_shaft");
  this->InsertMesh("axis_head");

  // Force visual
  dPtr->forceVisual.reset(new rendering::Visual(
      this->Name() + "_FORCE_VISUAL_", shared_from_this(), false));
  dPtr->forceVisual->Load();

  // Force shaft
  VisualPtr forceShaftVisual(new rendering::Visual(
      this->Name() + "_FORCE_SHAFT_", dPtr->forceVisual, false));
  forceShaftVisual->Load();

  forceShaftVisual->AttachMesh("axis_shaft");
  Ogre::MovableObject *shaftObj =
      forceShaftVisual->GetSceneNode()->getAttachedObject(0);
  shaftObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
  shaftObj->getUserObjectBindings().setUserAny(
        Ogre::Any(std::string(dPtr->forceVisual->Name())));
  forceShaftVisual->SetPosition(ignition::math::Vector3d(0, 0, 0.1));

  // Force head
  VisualPtr forceHeadVisual(new rendering::Visual(
      this->Name() + "_FORCE_HEAD_", dPtr->forceVisual, false));
  forceHeadVisual->Load();

  forceHeadVisual->AttachMesh("axis_head");
  Ogre::MovableObject *headObj =
      forceHeadVisual->GetSceneNode()->getAttachedObject(0);
  headObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
  headObj->getUserObjectBindings().setUserAny(
        Ogre::Any(std::string(dPtr->forceVisual->Name())));
  forceHeadVisual->SetPosition(ignition::math::Vector3d(0, 0, 0.24));

  dPtr->forceVisual->SetMaterial(dPtr->unselectedMaterial);
  dPtr->forceVisual->GetSceneNode()->setInheritScale(false);

  // Force text
  ignition::math::Color matAmbient, matDiffuse, matSpecular, matEmissive;
  rendering::Material::MaterialAsColor(dPtr->unselectedMaterial,
      matAmbient, matDiffuse, matSpecular, matEmissive);
  dPtr->forceText.Load(this->Name()+"__FORCE_TEXT__",
      "0N", "Arial", 0.03, matAmbient);
  dPtr->forceText.SetShowOnTop(true);

  dPtr->forceText.MovableObject::getUserObjectBindings().setUserAny(
      Ogre::Any(std::string(dPtr->forceVisual->Name())));

  VisualPtr forceTextVisual(new rendering::Visual(
      this->Name() + "_FORCE_TEXT_", dPtr->forceVisual, false));
  forceTextVisual->Load();
  forceTextVisual->GetSceneNode()->attachObject(&(dPtr->forceText));
  forceTextVisual->GetSceneNode()->setInheritScale(false);

  // Torque visual
  dPtr->torqueVisual.reset(new rendering::Visual(
      this->Name() + "_TORQUE_VISUAL_", shared_from_this(), false));
  dPtr->torqueVisual->Load();

  // Torque tube
  common::MeshManager::Instance()->CreateTube("torque_tube",
      0.1, 0.15, 0.05, 2, 32, 1.5*M_PI);
  this->InsertMesh("torque_tube");

  VisualPtr torqueTubeVisual(new rendering::Visual(
      this->Name() + "_TORQUE_TUBE_", dPtr->torqueVisual, false));
  torqueTubeVisual->Load();

  torqueTubeVisual->AttachMesh("torque_tube");
  Ogre::MovableObject *tubeObj =
      torqueTubeVisual->GetSceneNode()->getAttachedObject(0);
  tubeObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
  tubeObj->getUserObjectBindings().setUserAny(
        Ogre::Any(std::string(dPtr->torqueVisual->Name())));

  // Torque arrow
  VisualPtr torqueArrowVisual(new rendering::Visual(
      this->Name() + "_TORQUE_HEAD_", dPtr->torqueVisual, false));
  torqueArrowVisual->Load();

  torqueArrowVisual->AttachMesh("axis_head");
  Ogre::MovableObject *torqueHeadObj =
      torqueArrowVisual->GetSceneNode()->getAttachedObject(0);
  torqueHeadObj->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
  torqueHeadObj->getUserObjectBindings().setUserAny(
        Ogre::Any(std::string(dPtr->torqueVisual->Name())));

  torqueArrowVisual->SetScale(ignition::math::Vector3d(3, 3, 1));
  torqueArrowVisual->SetPosition(ignition::math::Vector3d(-0.04, 0.125, 0));
  ignition::math::Quaterniond quat(0, -M_PI/2.0, 0);
  torqueArrowVisual->SetRotation(quat);

  dPtr->torqueVisual->SetMaterial(dPtr->unselectedMaterial);
  dPtr->torqueVisual->GetSceneNode()->setInheritScale(false);

  // Torque line
  dPtr->torqueLine = dPtr->torqueVisual->
      CreateDynamicLine(rendering::RENDERING_LINE_LIST);
  GZ_OGRE_SET_MATERIAL_BY_NAME(dPtr->torqueLine, dPtr->unselectedMaterial);
  dPtr->torqueLine->AddPoint(0, 0, 0);
  dPtr->torqueLine->AddPoint(0, 0, 0.1);

  // Torque text
  dPtr->torqueText.Load(this->Name()+"__TORQUE_TEXT__",
      "0Nm", "Arial", 0.03, matAmbient);
  dPtr->torqueText.SetShowOnTop(true);

  dPtr->torqueText.MovableObject::getUserObjectBindings().setUserAny(
      Ogre::Any(std::string(dPtr->torqueVisual->Name())));

  VisualPtr torqueTextVisual(new rendering::Visual(
      this->Name() + "_TORQUE_TEXT_", dPtr->torqueVisual, false));
  torqueTextVisual->Load();

  torqueTextVisual->GetSceneNode()->attachObject(&(dPtr->torqueText));
  torqueTextVisual->GetSceneNode()->setInheritScale(false);

  // Rotation manipulator
  dPtr->rotTool.reset(new rendering::SelectionObj(
      this->Name() + "__SELECTION_OBJ", shared_from_this()));
  dPtr->rotTool->Load();
  dPtr->rotTool->SetMode("rotate");
  dPtr->rotTool->SetHandleVisible(SelectionObj::ROT_X, false);
  dPtr->rotTool->SetHandleMaterial(SelectionObj::ROT_Y,
      "Gazebo/DarkMagentaTransparent");
  dPtr->rotTool->SetHandleMaterial(SelectionObj::ROT_Z,
      "Gazebo/DarkMagentaTransparent");

  // Initialize
  dPtr->forceVector = ignition::math::Vector3d::Zero;
  dPtr->torqueVector = ignition::math::Vector3d::Zero;

  this->SetVisibilityFlags(GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE);
  this->Resize();
  this->UpdateForceVisual();
  this->UpdateTorqueVisual();
  this->SetMode(Mode::NONE);
  this->SetInheritTransparency(false);
}

///////////////////////////////////////////////////
void ApplyWrenchVisual::SetCoM(const ignition::math::Vector3d &_comVector)
{
  ApplyWrenchVisualPrivate *dPtr =
      reinterpret_cast<ApplyWrenchVisualPrivate *>(this->dataPtr);

  dPtr->comVector = _comVector;

  {
    // UpdateTorqueVisual changes torqueVisual
    std::lock_guard<std::mutex> lock(dPtr->mutex);

    this->UpdateTorqueVisual();
  }
}

///////////////////////////////////////////////////
void ApplyWrenchVisual::SetForcePos(
    const ignition::math::Vector3d &_forcePosVector)
{
  ApplyWrenchVisualPrivate *dPtr =
      reinterpret_cast<ApplyWrenchVisualPrivate *>(this->dataPtr);

  dPtr->forcePosVector = _forcePosVector;

  {
    // UpdateForceVisual changes forceVisual
    std::lock_guard<std::mutex> lock(dPtr->mutex);

    this->UpdateForceVisual();
  }
}

///////////////////////////////////////////////////
void ApplyWrenchVisual::SetForce(const ignition::math::Vector3d &_forceVector,
    const bool _rotatedByMouse)
{
  ApplyWrenchVisualPrivate *dPtr =
      reinterpret_cast<ApplyWrenchVisualPrivate *>(this->dataPtr);

  std::ostringstream mag;
  mag << std::fixed << std::setprecision(3) << _forceVector.Length();
  dPtr->forceText.SetText(mag.str() + "N");

  dPtr->forceVector = _forceVector;
  dPtr->rotatedByMouse = _rotatedByMouse;

  if (_forceVector == ignition::math::Vector3d::Zero)
  {
    if (dPtr->torqueVector == ignition::math::Vector3d::Zero)
      this->SetMode(Mode::NONE);
    else
      this->SetMode(Mode::TORQUE);
  }
  else
  {
    this->SetMode(Mode::FORCE);
  }
}

///////////////////////////////////////////////////
void ApplyWrenchVisual::SetTorque(const ignition::math::Vector3d &_torqueVector,
    const bool _rotatedByMouse)
{
  ApplyWrenchVisualPrivate *dPtr =
      reinterpret_cast<ApplyWrenchVisualPrivate *>(this->dataPtr);

  std::ostringstream mag;
  mag << std::fixed << std::setprecision(3) << _torqueVector.Length();
  dPtr->torqueText.SetText(mag.str() + "Nm");

  dPtr->torqueVector = _torqueVector;
  dPtr->rotatedByMouse = _rotatedByMouse;

  if (_torqueVector == ignition::math::Vector3d::Zero)
  {
    if (dPtr->forceVector == ignition::math::Vector3d::Zero)
      this->SetMode(Mode::NONE);
    else
      this->SetMode(Mode::FORCE);
  }
  else
  {
    this->SetMode(Mode::TORQUE);
  }
}

///////////////////////////////////////////////////
void ApplyWrenchVisual::UpdateForceVisual()
{
  IGN_PROFILE("ApplyWrenchVisual::UpdateForceVisual");
  ApplyWrenchVisualPrivate *dPtr =
      reinterpret_cast<ApplyWrenchVisualPrivate *>(this->dataPtr);

  if (!dPtr->forceVisual || !dPtr->rotTool)
  {
    gzwarn << "No force visual" << std::endl;
    return;
  }

  ignition::math::Vector3d normVec = dPtr->forceVector;
  normVec.Normalize();

  // Place it on X axis in case it is zero
  if (normVec == ignition::math::Vector3d::Zero)
    normVec = ignition::math::Vector3d::UnitX;

  // Set rotation in the vector direction
  auto quat = ignition::math::Matrix4d::LookAt(ignition::math::Vector3d::Zero,
      normVec).Rotation();
  dPtr->forceVisual->SetRotation(quat * ignition::math::Quaterniond(
      ignition::math::Vector3d(0, M_PI/2.0, 0)));

  // Set arrow tip to forcePosVector
  dPtr->forceVisual->SetPosition(-normVec * 0.28 *
      dPtr->forceVisual->Scale().Z() + dPtr->forcePosVector);

  // Rotation tool
  dPtr->rotTool->SetPosition(dPtr->forcePosVector);
  if (!dPtr->rotatedByMouse)
    dPtr->rotTool->SetRotation(quat);
}

///////////////////////////////////////////////////
void ApplyWrenchVisual::UpdateTorqueVisual()
{
  IGN_PROFILE("ApplyWrenchVisual::UpdateTorqueVisual");
  ApplyWrenchVisualPrivate *dPtr =
      reinterpret_cast<ApplyWrenchVisualPrivate *>(this->dataPtr);

  if (!dPtr->torqueVisual || !dPtr->rotTool)
  {
    gzwarn << "No torque visual" << std::endl;
    return;
  }

  ignition::math::Vector3d normVec = dPtr->torqueVector;
  normVec.Normalize();

  // Place it on X axis in case it is zero
  if (normVec == ignition::math::Vector3d::Zero)
    normVec = ignition::math::Vector3d::UnitX;

  // Set rotation in the vector direction
  auto quat = ignition::math::Matrix4d::LookAt(ignition::math::Vector3d::Zero,
      normVec).Rotation();
  dPtr->torqueVisual->SetRotation(quat * ignition::math::Quaterniond(
      ignition::math::Vector3d(0, M_PI/2.0, 0)));

  // Position towards comVector
  double linkDiagonal = dPtr->parent->BoundingBox().Size().Length();
  dPtr->torqueVisual->SetPosition(normVec*linkDiagonal*0.75 + dPtr->comVector);
  dPtr->torqueLine->SetPoint(1,
      ignition::math::Vector3d(0, 0,
        -linkDiagonal*0.75) / dPtr->torqueVisual->Scale());

  // Rotation tool
  dPtr->rotTool->SetPosition(dPtr->comVector);
  if (!dPtr->rotatedByMouse)
    dPtr->rotTool->SetRotation(quat);
}

/////////////////////////////////////////////////
void ApplyWrenchVisual::Resize()
{
  IGN_PROFILE("ApplyWrenchVisual::Resize");
  ApplyWrenchVisualPrivate *dPtr =
      reinterpret_cast<ApplyWrenchVisualPrivate *>(this->dataPtr);

  if (!dPtr->parent || !dPtr->forceVisual || !dPtr->torqueVisual ||
      !dPtr->rotTool)
  {
    gzwarn << "ApplyWrenchVisual is incomplete." << std::endl;
    return;
  }

  // Protect force/torque visuals
  std::lock_guard<std::mutex> lock(dPtr->mutex);

  double linkSize = std::max(0.1, dPtr->parent->BoundingBox().Size().Length());

  // Force visual
  dPtr->forceVisual->SetScale(ignition::math::Vector3d(2*linkSize,
                                                       2*linkSize,
                                                       2*linkSize));

  // Torque visual
  dPtr->torqueVisual->SetScale(ignition::math::Vector3d(linkSize,
                                                        linkSize,
                                                        linkSize));

  // Rot tool
  dPtr->rotTool->SetScale(ignition::math::Vector3d(0.75*linkSize,
                                                   0.75*linkSize,
                                                   0.75*linkSize));

  // Texts
  double fontSize = 0.1*linkSize;
  dPtr->forceText.SetCharHeight(fontSize);
  dPtr->torqueText.SetCharHeight(fontSize);
  dPtr->forceText.SetBaseline(0.12*linkSize);
}

///////////////////////////////////////////////////
rendering::VisualPtr ApplyWrenchVisual::GetForceVisual() const
{
  ApplyWrenchVisualPrivate *dPtr =
      reinterpret_cast<ApplyWrenchVisualPrivate *>(this->dataPtr);

  if (!dPtr->forceVisual)
  {
    gzerr << "Force visual not found, but it should exist." << std::endl;
    return NULL;
  }

  std::lock_guard<std::mutex> lock(dPtr->mutex);

  return dPtr->forceVisual;
}

///////////////////////////////////////////////////
rendering::VisualPtr ApplyWrenchVisual::GetTorqueVisual() const
{
  ApplyWrenchVisualPrivate *dPtr =
      reinterpret_cast<ApplyWrenchVisualPrivate *>(this->dataPtr);

  if (!dPtr->torqueVisual)
  {
    gzerr << "Torque visual not found, but it should exist." << std::endl;
    return NULL;
  }

  std::lock_guard<std::mutex> lock(dPtr->mutex);

  return dPtr->torqueVisual;
}

///////////////////////////////////////////////////
rendering::SelectionObjPtr ApplyWrenchVisual::GetRotTool() const
{
  ApplyWrenchVisualPrivate *dPtr =
      reinterpret_cast<ApplyWrenchVisualPrivate *>(this->dataPtr);

  if (!dPtr->rotTool)
  {
    gzerr << "Rot tool not found, but it should exist." << std::endl;
    return NULL;
  }

  return dPtr->rotTool;
}

/////////////////////////////////////////////////
void ApplyWrenchVisual::SetMode(Mode _mode)
{
  ApplyWrenchVisualPrivate *dPtr =
      reinterpret_cast<ApplyWrenchVisualPrivate *>(this->dataPtr);

  if (!dPtr->forceVisual || !dPtr->torqueVisual || !dPtr->rotTool)
  {
    gzerr << "Some visual is missing!" << std::endl;
    return;
  }

  // Protect force/torque visuals
  std::lock_guard<std::mutex> lock(dPtr->mutex);

  if (_mode == Mode::FORCE)
  {
    dPtr->forceVisual->SetMaterial(dPtr->selectedMaterial);
    dPtr->torqueVisual->SetMaterial(dPtr->unselectedMaterial);

    dPtr->rotTool->SetHandleVisible(SelectionObj::ROT_Y, true);
    dPtr->rotTool->SetHandleVisible(SelectionObj::ROT_Z, true);

    this->UpdateForceVisual();
  }
  else if (_mode == Mode::TORQUE)
  {
    dPtr->torqueVisual->SetMaterial(dPtr->selectedMaterial);
    dPtr->forceVisual->SetMaterial(dPtr->unselectedMaterial);

    dPtr->rotTool->SetHandleVisible(SelectionObj::ROT_Y, true);
    dPtr->rotTool->SetHandleVisible(SelectionObj::ROT_Z, true);

    this->UpdateTorqueVisual();
  }
  else if (_mode == Mode::NONE)
  {
    // Dark visuals
    dPtr->forceVisual->SetMaterial(dPtr->unselectedMaterial);
    dPtr->torqueVisual->SetMaterial(dPtr->unselectedMaterial);
    // hide rot
    dPtr->rotTool->SetHandleVisible(SelectionObj::ROT_Y, false);
    dPtr->rotTool->SetHandleVisible(SelectionObj::ROT_Z, false);
  }
}
