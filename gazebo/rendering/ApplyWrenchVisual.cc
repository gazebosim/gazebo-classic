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

  // Point visual
  math::Vector3 p1(0, 0, -2*linkSize.z);
  math::Vector3 p2(0, 0,  2*linkSize.z);
  math::Vector3 p3(0, -2*linkSize.y, 0);
  math::Vector3 p4(0,  2*linkSize.y, 0);
  math::Vector3 p5(-2*linkSize.x, 0, 0);
  math::Vector3 p6(2*linkSize.x,  0, 0);

  dPtr->crossLines = this->
      CreateDynamicLine(rendering::RENDERING_LINE_LIST);
  dPtr->crossLines->setMaterial("Gazebo/SkyBlue");
  dPtr->crossLines->AddPoint(p1);
  dPtr->crossLines->AddPoint(p2);
  dPtr->crossLines->AddPoint(p3);
  dPtr->crossLines->AddPoint(p4);
  dPtr->crossLines->AddPoint(p5);
  dPtr->crossLines->AddPoint(p6);

  // Force visual
  dPtr->forceVisual.reset(new rendering::ArrowVisual(
      this->GetName() + "__FORCE_VISUAL__", shared_from_this()));
  dPtr->forceVisual->Load();
  dPtr->forceVisual->SetMaterial("Gazebo/RedBright");
  dPtr->forceVisual->SetScale(math::Vector3(2, 2, 2));
  dPtr->forceVisual->GetSceneNode()->setInheritScale(false);

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
  double linkDiagonal = dPtr->parent->GetBoundingBox().GetDiagonalLength();
  dPtr->torqueLine = dPtr->torqueVisual->
      CreateDynamicLine(rendering::RENDERING_LINE_LIST);
  dPtr->torqueLine->setMaterial("Gazebo/Orange");
  dPtr->torqueLine->AddPoint(0, 0, 0);
  dPtr->torqueLine->AddPoint(0, 0, linkDiagonal*0.5 + 0.5);

  // Rotation manipulator
  dPtr->rotTool.reset(new rendering::SelectionObj(
      this->GetName() + "__SELECTION_OBJ", shared_from_this()));
  dPtr->rotTool->Load();
  dPtr->rotTool->SetMode("rotate");

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
void ApplyWrenchVisual::SetMode(WrenchModes _mode)
{
//  ApplyWrenchVisualPrivate *dPtr =
//      reinterpret_cast<ApplyWrenchVisualPrivate *>(this->dataPtr);

  this->wrenchMode = _mode;

//  if (this->wrenchMode == WrenchModes::FORCE)
//  {
//    dPtr->forceVisual->SetVisible(true);
//    dPtr->torqueVisual->SetVisible(false);
//  }
//  else if (this->wrenchMode == WrenchModes::TORQUE)
//  {
//    dPtr->forceVisual->SetVisible(false);
//    dPtr->torqueVisual->SetVisible(true);
//  }
}

///////////////////////////////////////////////////
void ApplyWrenchVisual::UpdateForce(math::Vector3 _forceVector)
{
  ApplyWrenchVisualPrivate *dPtr =
      reinterpret_cast<ApplyWrenchVisualPrivate *>(this->dataPtr);

  if (!dPtr->forceVisual)
    return;

  if (_forceVector == math::Vector3::Zero ||
      this->wrenchMode != WrenchModes::FORCE)
  {
    dPtr->forceVisual->SetVisible(false);
    return;
  }
  dPtr->forceVisual->SetVisible(true);

  // Set rotation
  math::Vector3 u = _forceVector;
  u = u.Normalize();
  math::Vector3 v = math::Vector3::UnitX;
  double cosTheta = v.Dot(u);
  double angle = acos(cosTheta);
  math::Quaternion quat;
  if (math::equal(angle, M_PI))
    quat.SetFromAxis(u.GetPerpendicular(), angle);
  else
    quat.SetFromAxis((v.Cross(u)).Normalize(), angle);

  dPtr->forceVisual->SetRotation(quat * math::Quaternion(
      math::Vector3(0, M_PI/2.0, 0)));

  // Set position
  double linkDiagonal = dPtr->parent->GetBoundingBox().GetDiagonalLength();
  //double arrowSize = this->forceVisual->GetBoundingBox().GetZLength();
  dPtr->forceVisual->SetPosition(-u * (linkDiagonal*0.5 + 0.5));

  // Rotation tool
  dPtr->rotTool->SetRotation(quat);
}

///////////////////////////////////////////////////
void ApplyWrenchVisual::UpdateTorque(math::Vector3 _torqueVector)
{
  ApplyWrenchVisualPrivate *dPtr =
      reinterpret_cast<ApplyWrenchVisualPrivate *>(this->dataPtr);

  if (!dPtr->torqueVisual)
    return;

  if (_torqueVector == math::Vector3::Zero ||
      this->wrenchMode != WrenchModes::TORQUE)
  {
    dPtr->torqueVisual->SetVisible(false);
    return;
  }
  dPtr->torqueVisual->SetVisible(true);

  // Set rotation
  math::Vector3 u = _torqueVector;
  u = u.Normalize();
  math::Vector3 v = math::Vector3::UnitZ;
  double cosTheta = v.Dot(u);
  double angle = acos(cosTheta);
  math::Quaternion quat;
  if (math::equal(angle, M_PI))
    quat.SetFromAxis(u.GetPerpendicular(), angle);
  else
    quat.SetFromAxis((v.Cross(u)).Normalize(), angle);
  dPtr->torqueVisual->SetRotation(quat);

  // Set position
  double linkDiagonal = dPtr->parent->GetBoundingBox().GetDiagonalLength();
  dPtr->torqueVisual->SetPosition(-u * (linkDiagonal*0.5 + 0.5));

  // Rotation tool
  dPtr->rotTool->SetRotation(quat);
}
