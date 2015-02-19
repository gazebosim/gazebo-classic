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

#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/MovableText.hh"
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

  // Origin visual
  dPtr->originVisual.reset(new rendering::Visual(
       this->GetName() + "__ORIGIN_VISUAL__", shared_from_this()));
  dPtr->originVisual->Load();

  dPtr->originXLines = dPtr->originVisual->
      CreateDynamicLine(rendering::RENDERING_LINE_LIST);
  dPtr->originXLines->setMaterial("Gazebo/RedTransparent");
  dPtr->originXLines->AddPoint(math::Vector3::Zero);
  dPtr->originXLines->AddPoint(math::Vector3::Zero);

  dPtr->originYLines = dPtr->originVisual->
      CreateDynamicLine(rendering::RENDERING_LINE_LIST);
  dPtr->originYLines->setMaterial("Gazebo/GreenTransparent");
  dPtr->originYLines->AddPoint(math::Vector3::Zero);
  dPtr->originYLines->AddPoint(math::Vector3::Zero);

  dPtr->originZLines = dPtr->originVisual->
      CreateDynamicLine(rendering::RENDERING_LINE_LIST);
  dPtr->originZLines->setMaterial("Gazebo/BlueTransparent");
  dPtr->originZLines->AddPoint(math::Vector3::Zero);
  dPtr->originZLines->AddPoint(math::Vector3::Zero);

  // CoM visual
  dPtr->comVisual.reset(new rendering::Visual(
       this->GetName() + "__CoM_VISUAL__", shared_from_this()));
  dPtr->comVisual->Load();

  this->InsertMesh("unit_sphere");

  Ogre::MovableObject *comObj =
    (Ogre::MovableObject*)(dPtr->scene->GetManager()->createEntity(
          this->GetName()+"__CoM_SPHERE__", "unit_sphere"));
  comObj->setCastShadows(false);

  Ogre::SceneNode *comNode =
      dPtr->comVisual->GetSceneNode()->createChildSceneNode(
      this->GetName() + "__COM_SPHERE_NODE__");

  comNode->attachObject(comObj);
  math::Quaternion rot(1.57, 0, 0);
  comNode->setOrientation(
      Ogre::Quaternion(rot.w, rot.x, rot.y, rot.z));
  dPtr->comVisual->SetMaterial("Gazebo/CoM");
  dPtr->comVisual->GetSceneNode()->setInheritScale(false);

  // Force visual
  dPtr->forceVisual.reset(new rendering::Visual(
      this->GetName() + "__FORCE_VISUAL__", shared_from_this()));
  dPtr->forceVisual->Load();
  dPtr->scene->AddVisual(dPtr->forceVisual);

  // Force shaft
  this->InsertMesh("axis_shaft");

  Ogre::MovableObject *shaftObj =
    (Ogre::MovableObject*)(dPtr->scene->GetManager()->createEntity(
          this->GetName()+"__FORCE_SHAFT__", "axis_shaft"));
  shaftObj->getUserObjectBindings().setUserAny(
      Ogre::Any(std::string(dPtr->forceVisual->GetName())));

  Ogre::SceneNode *shaftNode =
      dPtr->forceVisual->GetSceneNode()->createChildSceneNode(
      this->GetName() + "__FORCE_SHAFT_NODE__");
  shaftNode->attachObject(shaftObj);
  shaftNode->setPosition(0, 0, 0.1);

  // Force head
  this->InsertMesh("axis_head");

  Ogre::MovableObject *headObj =
    (Ogre::MovableObject*)(dPtr->scene->GetManager()->createEntity(
          this->GetName()+"__FORCE_HEAD__", "axis_head"));
  headObj->getUserObjectBindings().setUserAny(
      Ogre::Any(std::string(dPtr->forceVisual->GetName())));

  Ogre::SceneNode *headNode =
      dPtr->forceVisual->GetSceneNode()->createChildSceneNode(
      this->GetName() + "__FORCE_HEAD_NODE__");
  headNode->attachObject(headObj);
  headNode->setPosition(0, 0, 0.24);

  dPtr->forceVisual->SetMaterial("Gazebo/OrangeTransparentOverlay");
  dPtr->forceVisual->GetSceneNode()->setInheritScale(false);

  // Force text
  dPtr->forceText = new MovableText();
  dPtr->forceText->Load(this->GetName()+"__FORCE_TEXT__",
      "0N", "Arial", 0.03, common::Color(1, 0.6, 0));
  dPtr->forceText->SetShowOnTop(true);

  dPtr->forceText->MovableObject::getUserObjectBindings().setUserAny(
      Ogre::Any(std::string(dPtr->forceVisual->GetName())));

  Ogre::SceneNode *forceTextNode =
      dPtr->forceVisual->GetSceneNode()->createChildSceneNode(
      this->GetName() + "__FORCE_TEXT_NODE__");
  forceTextNode->attachObject(dPtr->forceText);
  forceTextNode->setInheritScale(false);

  // Torque visual
  dPtr->torqueVisual.reset(new rendering::Visual(
       this->GetName() + "__TORQUE_VISUAL__", shared_from_this()));
  dPtr->torqueVisual->Load();
  dPtr->scene->AddVisual(dPtr->torqueVisual);

  // Torque tube
  common::MeshManager::Instance()->CreateTube("torque_tube",
      0.1, 0.15, 0.05, 2, 32, 1.5*M_PI);
  this->InsertMesh("torque_tube");

  Ogre::MovableObject *tubeObj =
    (Ogre::MovableObject*)(dPtr->scene->GetManager()->createEntity(
        this->GetName()+"__TORQUE_TUBE__", "torque_tube"));
  tubeObj->getUserObjectBindings().setUserAny(
      Ogre::Any(std::string(dPtr->torqueVisual->GetName())));

  Ogre::SceneNode *tubeNode =
      dPtr->torqueVisual->GetSceneNode()->createChildSceneNode(
      this->GetName() + "__TORQUE_TUBE_NODE__");
  tubeNode->attachObject(tubeObj);

  // Torque arrow
  this->InsertMesh("axis_head");

  Ogre::MovableObject *torqueHeadObj =
    (Ogre::MovableObject*)(dPtr->scene->GetManager()->createEntity(
          this->GetName()+"__TORQUE_HEAD__", "axis_head"));
  torqueHeadObj->getUserObjectBindings().setUserAny(
      Ogre::Any(std::string(dPtr->torqueVisual->GetName())));

  Ogre::SceneNode *torqueHeadNode =
      dPtr->torqueVisual->GetSceneNode()->createChildSceneNode(
      this->GetName() + "__TORQUE_HEAD_NODE__");
  torqueHeadNode->attachObject(torqueHeadObj);
  torqueHeadNode->setScale(3, 3, 1);
  torqueHeadNode->setPosition(-0.04, 0.125, 0);
  math::Quaternion quat(0, -1.57, 0);
  torqueHeadNode->setOrientation(
      Ogre::Quaternion(quat.w, quat.x, quat.y, quat.z));

  dPtr->torqueVisual->SetMaterial("Gazebo/OrangeTransparentOverlay");
  dPtr->torqueVisual->GetSceneNode()->setInheritScale(false);

  // Torque line
  dPtr->torqueLine = dPtr->torqueVisual->
      CreateDynamicLine(rendering::RENDERING_LINE_LIST);
  dPtr->torqueLine->setMaterial("Gazebo/OrangeTransparentOverlay");
  dPtr->torqueLine->AddPoint(0, 0, 0);
  dPtr->torqueLine->AddPoint(0, 0, 0.1);

  // Torque text
  dPtr->torqueText = new MovableText();
  dPtr->torqueText->Load(this->GetName()+"__TORQUE_TEXT__",
      "0N", "Arial", 0.03, common::Color(1, 0.6, 0));
  dPtr->torqueText->SetShowOnTop(true);

  dPtr->torqueText->MovableObject::getUserObjectBindings().setUserAny(
      Ogre::Any(std::string(dPtr->torqueVisual->GetName())));

  Ogre::SceneNode *torqueTextNode =
      dPtr->torqueVisual->GetSceneNode()->createChildSceneNode(
      this->GetName() + "__TORQUE_TEXT_NODE__");
  torqueTextNode->attachObject(dPtr->torqueText);
  torqueTextNode->setInheritScale(false);

  // Rotation manipulator
  dPtr->rotTool.reset(new rendering::SelectionObj(
      this->GetName() + "__SELECTION_OBJ", shared_from_this()));
  dPtr->rotTool->Load();
  dPtr->rotTool->SetMode("rotate");
  dPtr->rotTool->SetHandleVisible(SelectionObj::ROT_X, false);
  dPtr->rotTool->SetHandleMaterial(SelectionObj::ROT_Y, "Gazebo/DarkMagentaTransparent");
  dPtr->rotTool->SetHandleMaterial(SelectionObj::ROT_Z, "Gazebo/DarkMagentaTransparent");

  // Initialize
  dPtr->forceVector = math::Vector3::Zero;
  dPtr->torqueVector = math::Vector3::Zero;
  dPtr->mode = "force";

  this->SetVisibilityFlags(GZ_VISIBILITY_GUI |
      GZ_VISIBILITY_SELECTABLE);
  this->Resize();
  this->SetForceVisual();
  this->SetTorqueVisual();
  this->SetWrenchMode("none");
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

  if (!dPtr->forceVisual || !dPtr->torqueVisual || !dPtr->rotTool)
  {
    gzwarn << "Some visual is missing!" << std::endl;
    return;
  }

  // Update variable
  dPtr->mode = _mode;

  if (_mode == "force")
  {
    dPtr->forceVisual->SetMaterial("Gazebo/OrangeTransparentOverlay");
    dPtr->torqueVisual->SetMaterial("Gazebo/DarkOrangeTransparentOverlay");

    dPtr->rotTool->SetHandleVisible(SelectionObj::ROT_Y, true);
    dPtr->rotTool->SetHandleVisible(SelectionObj::ROT_Z, true);

    // set force visual visible and update direction
    this->SetForceVisual();
  }
  else if (_mode == "torque")
  {
    dPtr->torqueVisual->SetMaterial("Gazebo/OrangeTransparentOverlay");
    dPtr->forceVisual->SetMaterial("Gazebo/DarkOrangeTransparentOverlay");

    dPtr->rotTool->SetHandleVisible(SelectionObj::ROT_Y, true);
    dPtr->rotTool->SetHandleVisible(SelectionObj::ROT_Z, true);

    // set torque visual visible and update direction
    this->SetTorqueVisual();
  }
  else if (_mode == "none")
  {
    // Dark visuals
    dPtr->forceVisual->SetMaterial("Gazebo/DarkOrangeTransparentOverlay");
    dPtr->torqueVisual->SetMaterial("Gazebo/DarkOrangeTransparentOverlay");
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

  this->Resize();

  dPtr->comVector = _comVector;
  this->SetTorqueVisual();
}

///////////////////////////////////////////////////
void ApplyWrenchVisual::SetForcePos(math::Vector3 _forcePosVector)
{
  ApplyWrenchVisualPrivate *dPtr =
      reinterpret_cast<ApplyWrenchVisualPrivate *>(this->dataPtr);

  dPtr->forcePosVector = _forcePosVector;
  this->SetForceVisual();
}

///////////////////////////////////////////////////
void ApplyWrenchVisual::SetForce(math::Vector3 _forceVector, bool _rotatedByMouse)
{
  ApplyWrenchVisualPrivate *dPtr =
      reinterpret_cast<ApplyWrenchVisualPrivate *>(this->dataPtr);

  dPtr->forceVector = _forceVector;
  dPtr->rotatedByMouse = _rotatedByMouse;

  std::ostringstream mag;
  mag << std::fixed << std::setprecision(3) << _forceVector.GetLength();
  dPtr->forceText->SetText(mag.str() + "N");

  if (_forceVector == math::Vector3::Zero)
  {
//    dPtr->forceVisual->SetVisible(false);
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

  std::ostringstream mag;
  mag << std::fixed << std::setprecision(3) << _torqueVector.GetLength();
  dPtr->torqueText->SetText(mag.str() + "Nm");

  if (_torqueVector == math::Vector3::Zero)
  {
//    dPtr->torqueVisual->SetVisible(false);
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

  if (!dPtr->forceVisual)
  {
    gzwarn << "No force visual" << std::endl;
    return;
  }

  // position according to force and position vectors

  // Place it on X axis in case it is zero
  math::Vector3 normVec = dPtr->forceVector;
  normVec.Normalize();

  if (normVec == math::Vector3::Zero)
    normVec = math::Vector3::UnitX;

  // Set rotation
  math::Quaternion quat = this->GetQuaternionFromVector(normVec);
  dPtr->forceVisual->SetRotation(quat * math::Quaternion(
      math::Vector3(0, M_PI/2.0, 0)));

  // Set position towards forcePosVector
  dPtr->forceVisual->SetPosition(-normVec * 0.28 * dPtr->forceVisual->GetScale().z
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

  if (!dPtr->torqueVisual)
  {
    gzwarn << "No torque visual" << std::endl;
    return;
  }

  // position according to torque and position vectors

  // Place it on X axis in case it is zero
  math::Vector3 normVec = dPtr->torqueVector;
  normVec.Normalize();

  if (normVec == math::Vector3::Zero)
    normVec = math::Vector3::UnitX;

  // Set rotation
  math::Quaternion quat = this->GetQuaternionFromVector(normVec);
  dPtr->torqueVisual->SetRotation(quat * math::Quaternion(
      math::Vector3(0, M_PI/2.0, 0)));

  // Set position towards comVector
  double linkDiagonal = dPtr->parent->GetBoundingBox().GetSize().GetLength();
  dPtr->torqueVisual->SetPosition(normVec * linkDiagonal*0.75
      + dPtr->comVector);
  dPtr->torqueLine->SetPoint(1,
      math::Vector3(0, 0, -linkDiagonal*0.75)/dPtr->torqueVisual->GetScale());

  // Rotation tool
  dPtr->rotTool->SetPosition(dPtr->comVector);
  if (!dPtr->rotatedByMouse)
    dPtr->rotTool->SetRotation(quat);
}

/////////////////////////////////////////////////
void ApplyWrenchVisual::SetVisible(bool _visible, bool _cascade)
{
  ApplyWrenchVisualPrivate *dPtr =
      reinterpret_cast<ApplyWrenchVisualPrivate *>(this->dataPtr);

  if (!dPtr->forceVisual || !dPtr->torqueVisual || !dPtr->comVisual ||
      !dPtr->rotTool)
  {
    gzwarn << "Some visual is missing!" << std::endl;
    return;
  }

  if (_visible)
  {
    dPtr->comVisual->SetVisible(true);
    dPtr->originVisual->SetVisible(true);
    dPtr->forceVisual->SetVisible(true);
    dPtr->torqueVisual->SetVisible(true);

    if (dPtr->mode != "none")
    {
      dPtr->rotTool->SetHandleVisible(SelectionObj::ROT_Y, true);
      dPtr->rotTool->SetHandleVisible(SelectionObj::ROT_Z, true);
      if (dPtr->mode == "force")
        dPtr->forceVisual->SetMaterial("Gazebo/OrangeTransparentOverlay");
      else
        dPtr->torqueVisual->SetMaterial("Gazebo/OrangeTransparentOverlay");
    }
  }
  else
  {
    dPtr->comVisual->SetVisible(false);
    dPtr->originVisual->SetVisible(false);
    dPtr->rotTool->SetHandleVisible(SelectionObj::ROT_Y, false);
    dPtr->rotTool->SetHandleVisible(SelectionObj::ROT_Z, false);

    // Use cascade to hide mode visuals or not
    if (_cascade)
    {
      dPtr->forceVisual->SetVisible(false);
      dPtr->torqueVisual->SetVisible(false);
    }
    else
    {
      dPtr->forceVisual->SetMaterial("Gazebo/DarkOrangeTransparentOverlay");
      dPtr->torqueVisual->SetMaterial("Gazebo/DarkOrangeTransparentOverlay");
    }
  }
}

/////////////////////////////////////////////////
void ApplyWrenchVisual::Resize()
{
  ApplyWrenchVisualPrivate *dPtr =
      reinterpret_cast<ApplyWrenchVisualPrivate *>(this->dataPtr);

  math::Vector3 linkSize = dPtr->parent->GetBoundingBox().GetSize();

  // Origin visual
  dPtr->originXLines->SetPoint(0, math::Vector3(-linkSize.x, 0, 0));
  dPtr->originXLines->SetPoint(1, math::Vector3( linkSize.x,  0, 0));
  dPtr->originYLines->SetPoint(0, math::Vector3(0, -linkSize.y, 0));
  dPtr->originYLines->SetPoint(1, math::Vector3(0,  linkSize.y, 0));
  dPtr->originZLines->SetPoint(0, math::Vector3(0, 0, -linkSize.z));
  dPtr->originZLines->SetPoint(1, math::Vector3(0, 0,  linkSize.z));

  // CoM visual
  double comScale = linkSize.GetMin() * 0.1;
  dPtr->comVisual->SetScale(math::Vector3(comScale, comScale, comScale));

  // Force visual
  dPtr->forceVisual->SetScale(math::Vector3(2*linkSize.GetLength(),
                                            2*linkSize.GetLength(),
                                            2*linkSize.GetLength()));

  // Torque visual
  dPtr->torqueVisual->SetScale(math::Vector3(linkSize.GetLength(),
                                             linkSize.GetLength(),
                                             linkSize.GetLength()));

  // Texts
  double fontSize = std::max(0.1*linkSize.GetLength(), 0.1);
  dPtr->forceText->SetCharHeight(fontSize);
  dPtr->torqueText->SetCharHeight(fontSize);
  dPtr->forceText->SetBaseline(0.12*linkSize.GetLength());

  // Rot tool
  dPtr->rotTool->SetScale(math::Vector3(0.75*linkSize.GetLength(),
                                        0.75*linkSize.GetLength(),
                                        0.75*linkSize.GetLength()));
}
