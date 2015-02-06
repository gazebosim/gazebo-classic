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
  dPtr->rotTool->SetHandleVisible("rotate", 0, false);

  dPtr->forceVector = math::Vector3::UnitX;
  dPtr->torqueVector = math::Vector3::Zero;

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
  ApplyWrenchVisualPrivate *dPtr =
      reinterpret_cast<ApplyWrenchVisualPrivate *>(this->dataPtr);

  this->wrenchMode = _mode;

  // Attach rotation to mode visual
  dPtr->rotTool->SetHandleVisible("rotate", 1, true);
  dPtr->rotTool->SetHandleVisible("rotate", 2, true);
  if (this->wrenchMode == WrenchModes::FORCE &&
      dPtr->forceVector != math::Vector3::Zero)
  {
//    dPtr->rotTool->SetRotation(dPtr->forceVisual->GetRotation() *
//        math::Quaternion(math::Vector3(0, M_PI/2.0, 0)));
  }
  else if (this->wrenchMode == WrenchModes::TORQUE &&
      dPtr->torqueVector != math::Vector3::Zero)
  {
//    dPtr->rotTool->SetRotation(dPtr->torqueVisual->GetRotation() *
//        math::Quaternion(math::Vector3(0, M_PI/2.0, 0)));
  }
  else if (dPtr->forceVector == math::Vector3::Zero &&
           dPtr->torqueVector == math::Vector3::Zero)
  {
    dPtr->rotTool->SetHandleVisible("rotate", 1, false);
    dPtr->rotTool->SetHandleVisible("rotate", 2, false);
  }
}

///////////////////////////////////////////////////
void ApplyWrenchVisual::UpdateForce(math::Vector3 _forceVector, bool _rotateTool)
{
  ApplyWrenchVisualPrivate *dPtr =
      reinterpret_cast<ApplyWrenchVisualPrivate *>(this->dataPtr);

  if (!dPtr->forceVisual)
    return;

  dPtr->forceVector = _forceVector;

  if (_forceVector == math::Vector3::Zero)
  {
    dPtr->forceVisual->SetVisible(false);
    return;
  }
  dPtr->forceVisual->SetVisible(true);

  // Set rotation
  math::Vector3 normVec = _forceVector;
  normVec.Normalize();
  math::Quaternion quat = this->GetQuaternionFromVector(normVec);
  dPtr->forceVisual->SetRotation(quat * math::Quaternion(
      math::Vector3(0, M_PI/2.0, 0)));

  // Set position
  double linkDiagonal = dPtr->parent->GetBoundingBox().GetDiagonalLength();
  //double arrowSize = this->forceVisual->GetBoundingBox().GetZLength();
  dPtr->forceVisual->SetPosition(-normVec * (linkDiagonal*0.5 + 0.5));

  // Rotation tool
  if (_rotateTool)
    dPtr->rotTool->SetRotation(quat);
}

///////////////////////////////////////////////////
void ApplyWrenchVisual::UpdateTorque(math::Vector3 _torqueVector, bool _rotateTool)
{
  ApplyWrenchVisualPrivate *dPtr =
      reinterpret_cast<ApplyWrenchVisualPrivate *>(this->dataPtr);

  if (!dPtr->torqueVisual)
    return;

  dPtr->torqueVector = _torqueVector;

  if (_torqueVector == math::Vector3::Zero)
  {
    dPtr->torqueVisual->SetVisible(false);
    return;
  }
  dPtr->torqueVisual->SetVisible(true);

  // Set rotation
  math::Vector3 normVec = _torqueVector;
  normVec.Normalize();
  math::Quaternion quat = this->GetQuaternionFromVector(normVec);
  dPtr->torqueVisual->SetRotation(quat * math::Quaternion(
      math::Vector3(0, M_PI/2.0, 0)));

  // Set position
  double linkDiagonal = dPtr->parent->GetBoundingBox().GetDiagonalLength();
  dPtr->torqueVisual->SetPosition(-normVec * (linkDiagonal*0.5 + 0.5));

  // Rotation tool
  if (_rotateTool)
    dPtr->rotTool->SetRotation(quat);
}

///////////////////////////////////////////////////
math::Quaternion ApplyWrenchVisual::GetQuaternionFromVector(math::Vector3 _vec)
{
//  _vec.Normalize();
//  math::Vector3 v = math::Vector3::UnitX;
//  double cosTheta = v.Dot(_vec);
//  double angle = acos(cosTheta);
//  math::Quaternion quat;
//  if (math::equal(angle, M_PI))
//    quat.SetFromAxis(_vec.GetPerpendicular(), angle);
//  else
//    quat.SetFromAxis((v.Cross(_vec)).Normalize(), angle);

//  return quat;

  // Adapted from
  // http://gamedev.stackexchange.com/questions/53129/quaternion-look-at-with-up-vector
  // Still doesn't stay properly up at some angles though.

//  math::Vector3 up = math::Vector3::UnitZ;

//  math::Vector3 forward_l = _vec;
//  math::Vector3 forward_w(1, 0, 0);
//  math::Vector3 axis  = forward_l.Cross(forward_w);
//  float angle = acos(forward_l.Dot(forward_w));

//  math::Vector3 third = axis.Cross(forward_w);
//  if (third.Dot(forward_l) < 0)
//  {
//    angle = -angle;
//  }
//  math::Quaternion q1;
//  q1.SetFromAxis(axis, angle);

//  math::Vector3 up_l  = q1 * up.Normalize();
//  math::Vector3 right = (forward_l.Cross(up)).Normalize();
//  math::Vector3 up_w  = (right.Cross(forward_l)).Normalize();

//  math::Vector3 axis2  = up_l.Cross(up_w);
//  float angle2 = acos(up_l.Dot(up_w));

////  math::Vector3 third2 = axis2.Cross(up_w);
////  if (third2.Dot(up_l) < 0)
////  {
////    angle2 = -angle2;
////  }
//  math::Quaternion q2;
//  q2.SetFromAxis(axis2, angle2);

//  return q2 * q1;

//  double roll = 0;
//  double pitch = -atan2(_vec.z, _vec.x);
//  double yaw = atan2(_vec.y, sqrt(pow(_vec.x, 2) + pow(_vec.z, 2)));

  double roll = 0;
  double pitch = -atan2(_vec.z, sqrt(pow(_vec.x, 2) + pow(_vec.y, 2)));
  double yaw = atan2(_vec.y, _vec.x);


  return math::Quaternion(roll, pitch, yaw);

}
