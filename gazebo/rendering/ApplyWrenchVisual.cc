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

#include "gazebo/common/MeshManager.hh"

//#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/ArrowVisual.hh"
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

///////////////////////////////////////////////////
void ApplyWrenchVisual::Load()
{
  ApplyWrenchVisualPrivate *dPtr =
      reinterpret_cast<ApplyWrenchVisualPrivate *>(this->dataPtr);

  math::Vector3 linkSize = dPtr->parent->GetBoundingBox().GetSize();

  // CoM visual
  dPtr->comVisual.reset(new rendering::Visual(
       this->GetName() + "__CoM_VISUAL__", shared_from_this()));
  dPtr->comVisual->Load();

  rendering::DynamicLines *comLines = dPtr->comVisual->
      CreateDynamicLine(rendering::RENDERING_LINE_LIST);
  comLines->setMaterial("Gazebo/SkyBlue");
  comLines->AddPoint(math::Vector3(0, 0, -2*linkSize.z));
  comLines->AddPoint(math::Vector3(0, 0,  2*linkSize.z));
  comLines->AddPoint(math::Vector3(0, -2*linkSize.y, 0));
  comLines->AddPoint(math::Vector3(0,  2*linkSize.y, 0));
  comLines->AddPoint(math::Vector3(-2*linkSize.x, 0, 0));
  comLines->AddPoint(math::Vector3(2*linkSize.x,  0, 0));

  // Force visual
  dPtr->forceVisual.reset(new rendering::ArrowVisual(
      this->GetName() + "__FORCE_VISUAL__", shared_from_this()));
  dPtr->forceVisual->Load();
  dPtr->forceVisual->SetMaterial("Gazebo/RedBright");
  dPtr->forceVisual->SetScale(math::Vector3(2, 2, 2));
  dPtr->forceVisual->GetSceneNode()->setInheritScale(false);

  // Force line
  double linkDiagonal = dPtr->parent->GetBoundingBox().GetSize().GetLength();
  rendering::DynamicLines *forceLine = dPtr->forceVisual->
      CreateDynamicLine(rendering::RENDERING_LINE_LIST);
  forceLine->setMaterial("Gazebo/RedBright");
  forceLine->AddPoint(0, 0, 0);
  forceLine->AddPoint(0, 0, linkDiagonal*0.5);

  // Torque visual
  // Torque tube
  dPtr->torqueVisual.reset(new rendering::Visual(
       this->GetName() + "__TORQUE_VISUAL__", shared_from_this()));
  dPtr->torqueVisual->Load();

  common::MeshManager::Instance()->CreateTube("torque_tube",
      0.1, 0.15, 0.05, 1, 32);
  this->InsertMesh("torque_tube");

  Ogre::MovableObject *torqueObj =
    (Ogre::MovableObject*)(dPtr->scene->GetManager()->createEntity(
        this->GetName()+"__TORQUE_VISUAL__", "torque_tube"));

  Ogre::SceneNode *torqueNode =
      dPtr->torqueVisual->GetSceneNode()->createChildSceneNode(
      this->GetName() + "__TORQUE_VISUAL_NODE__");
  torqueNode->attachObject(torqueObj);
  dPtr->torqueVisual->SetMaterial("Gazebo/Orange");

  // Torque line
  rendering::DynamicLines *torqueLine = dPtr->torqueVisual->
      CreateDynamicLine(rendering::RENDERING_LINE_LIST);
  torqueLine->setMaterial("Gazebo/Orange");
  torqueLine->AddPoint(0, 0, 0);
  torqueLine->AddPoint(0, 0, linkDiagonal*0.5 + 0.5);

  // Rotation manipulator
  dPtr->rotTool.reset(new rendering::SelectionObj(
      this->GetName() + "__SELECTION_OBJ", shared_from_this()));
  dPtr->rotTool->Load();
  dPtr->rotTool->SetMode("rotate");
  dPtr->rotTool->SetHandleVisible(SelectionObj::ROT_X, false);

  dPtr->forceVector = math::Vector3::UnitX;
  dPtr->torqueVector = math::Vector3::Zero;
  dPtr->mode = "force";

  this->SetVisibilityFlags(GZ_VISIBILITY_GUI);
}

///////////////////////////////////////////////////
rendering::VisualPtr ApplyWrenchVisual::GetForceVisual() const
{
  ApplyWrenchVisualPrivate *dPtr =
      reinterpret_cast<ApplyWrenchVisualPrivate *>(this->dataPtr);

  return dPtr->forceVisual;
}

///////////////////////////////////////////////////
rendering::VisualPtr ApplyWrenchVisual::GetTorqueVisual() const
{
  ApplyWrenchVisualPrivate *dPtr =
      reinterpret_cast<ApplyWrenchVisualPrivate *>(this->dataPtr);

  return dPtr->torqueVisual;
}

///////////////////////////////////////////////////
rendering::SelectionObjPtr ApplyWrenchVisual::GetRotTool() const
{
  ApplyWrenchVisualPrivate *dPtr =
      reinterpret_cast<ApplyWrenchVisualPrivate *>(this->dataPtr);

  return dPtr->rotTool;
}

///////////////////////////////////////////////////
math::Quaternion ApplyWrenchVisual::GetQuaternionFromVector(math::Vector3 _vec)
{
  double roll = 0;
  double pitch = -atan2(_vec.z, sqrt(pow(_vec.x, 2) + pow(_vec.y, 2)));
  double yaw = atan2(_vec.y, _vec.x);

  return math::Quaternion(roll, pitch, yaw);
}

/////////////////////////////////////////////////
void ApplyWrenchVisual::SetWrenchMode(std::string _mode)
{
  ApplyWrenchVisualPrivate *dPtr =
      reinterpret_cast<ApplyWrenchVisualPrivate *>(this->dataPtr);

  // Update variable
  dPtr->mode = _mode;

  if (_mode == "force")
  {
    // set force visual visible and update direction
    this->SetForceVisual();
  }
  else if (_mode == "torque")
  {
    // set torque visual visible and update direction
    this->SetTorqueVisual();
  }
  else if (_mode == "none")
  {
    // hide rot
    dPtr->rotTool->SetHandleVisible(SelectionObj::ROT_Y, false);
    dPtr->rotTool->SetHandleVisible(SelectionObj::ROT_Z, false);
  }
}

///////////////////////////////////////////////////
void ApplyWrenchVisual::SetCoM(math::Vector3 _comVector)
{
  ApplyWrenchVisualPrivate *dPtr =
      reinterpret_cast<ApplyWrenchVisualPrivate *>(this->dataPtr);

  // move com visual
  dPtr->comVisual->SetPosition(_comVector);

  dPtr->comVector = _comVector;
}

///////////////////////////////////////////////////
void ApplyWrenchVisual::SetForcePos(math::Vector3 _forcePosVector)
{
  ApplyWrenchVisualPrivate *dPtr =
      reinterpret_cast<ApplyWrenchVisualPrivate *>(this->dataPtr);

  if (dPtr->forceVector == math::Vector3::Zero)
    return;

  // move force
  dPtr->forceVisual->SetPosition(dPtr->forceVisual->GetPosition() -
      dPtr->forcePosVector + _forcePosVector);

  // move rot
  dPtr->rotTool->SetPosition(dPtr->rotTool->GetPosition() -
      dPtr->forcePosVector + _forcePosVector);

  dPtr->forcePosVector = _forcePosVector;
  this->SetWrenchMode("force");
}

///////////////////////////////////////////////////
void ApplyWrenchVisual::SetForce(math::Vector3 _forceVector, bool _rotatedByMouse)
{
  ApplyWrenchVisualPrivate *dPtr =
      reinterpret_cast<ApplyWrenchVisualPrivate *>(this->dataPtr);

  dPtr->forceVector = _forceVector;
  dPtr->rotatedByMouse = _rotatedByMouse;

  if (_forceVector == math::Vector3::Zero)
  {
    dPtr->forceVisual->SetVisible(false);
    if (dPtr->torqueVector == math::Vector3::Zero)
      this->SetWrenchMode("none");
    else
      this->SetWrenchMode("torque");
  }
  else
  {
    this->SetWrenchMode("force");
  }
}

///////////////////////////////////////////////////
void ApplyWrenchVisual::SetTorque(math::Vector3 _torqueVector, bool _rotatedByMouse)
{
  ApplyWrenchVisualPrivate *dPtr =
      reinterpret_cast<ApplyWrenchVisualPrivate *>(this->dataPtr);

  dPtr->torqueVector = _torqueVector;
  dPtr->rotatedByMouse = _rotatedByMouse;

  if (_torqueVector == math::Vector3::Zero)
  {
    dPtr->torqueVisual->SetVisible(false);
    if (dPtr->forceVector == math::Vector3::Zero)
      this->SetWrenchMode("none");
    else
      this->SetWrenchMode("force");
  }
  else
  {
    this->SetWrenchMode("torque");
  }
}

///////////////////////////////////////////////////
void ApplyWrenchVisual::SetForceVisual()
{
  ApplyWrenchVisualPrivate *dPtr =
      reinterpret_cast<ApplyWrenchVisualPrivate *>(this->dataPtr);

  // make visible
  dPtr->forceVisual->SetVisible(true);
  dPtr->forceVisual->SetMaterial("Gazebo/Red");
  dPtr->torqueVisual->SetMaterial("Gazebo/OrangeTransparent");
  dPtr->rotTool->SetHandleVisible(SelectionObj::ROT_Y, true);
  dPtr->rotTool->SetHandleVisible(SelectionObj::ROT_Z, true);

  // position according to force and position vectors

  // Set rotation
  math::Vector3 normVec = dPtr->forceVector;
  normVec.Normalize();
  math::Quaternion quat = this->GetQuaternionFromVector(normVec);
  dPtr->forceVisual->SetRotation(quat * math::Quaternion(
      math::Vector3(0, M_PI/2.0, 0)));

  // Set position
  double linkDiagonal = dPtr->parent->GetBoundingBox().GetSize().GetLength();
  //double arrowSize = this->forceVisual->GetBoundingBox().GetZLength();
  dPtr->forceVisual->SetPosition(-normVec * (linkDiagonal*0.5 + 0.5)
      + dPtr->forcePosVector);

  // Rotation tool
  dPtr->rotTool->SetPosition(dPtr->forcePosVector);
  if (!dPtr->rotatedByMouse)
    dPtr->rotTool->SetRotation(quat);
}

///////////////////////////////////////////////////
void ApplyWrenchVisual::SetTorqueVisual()
{
  ApplyWrenchVisualPrivate *dPtr =
      reinterpret_cast<ApplyWrenchVisualPrivate *>(this->dataPtr);

  // make visible
  dPtr->torqueVisual->SetVisible(true);
  dPtr->torqueVisual->SetMaterial("Gazebo/Orange");
  dPtr->forceVisual->SetMaterial("Gazebo/RedTransparent");
  dPtr->rotTool->SetHandleVisible(SelectionObj::ROT_Y, true);
  dPtr->rotTool->SetHandleVisible(SelectionObj::ROT_Z, true);

  // position according to torque and position vectors

  // Set rotation
  math::Vector3 normVec = dPtr->torqueVector;
  normVec.Normalize();
  math::Quaternion quat = this->GetQuaternionFromVector(normVec);
  dPtr->torqueVisual->SetRotation(quat * math::Quaternion(
      math::Vector3(0, M_PI/2.0, 0)));

  // Set position
  double linkDiagonal = dPtr->parent->GetBoundingBox().GetSize().GetLength();
  dPtr->torqueVisual->SetPosition(-normVec * (linkDiagonal*0.5 + 0.5));

  // Rotation tool
  dPtr->rotTool->SetPosition(math::Vector3::Zero);
  if (!dPtr->rotatedByMouse)
    dPtr->rotTool->SetRotation(quat);
}
