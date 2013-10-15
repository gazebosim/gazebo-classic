/*
 * Copyright 2013 Open Source Robotics Foundation
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
/* Desc: Center of Mass Visualization Class
 * Author: Nate Koenig
 */

#include "gazebo/common/MeshManager.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Quaternion.hh"
#include "gazebo/math/Pose.hh"

#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/COMVisual.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
COMVisual::COMVisual(const std::string &_name, VisualPtr _vis)
  : Visual(_name, _vis, false)
{
}

/////////////////////////////////////////////////
COMVisual::~COMVisual()
{
}

/////////////////////////////////////////////////
void COMVisual::Load(sdf::ElementPtr _elem)
{
  Visual::Load();
  math::Pose pose = _elem->Get<math::Pose>("origin");
  this->Load(pose);
}

/////////////////////////////////////////////////
void COMVisual::Load(ConstLinkPtr &_msg)
{
  Visual::Load();

  math::Vector3 xyz(_msg->inertial().pose().position().x(),
                    _msg->inertial().pose().position().y(),
                    _msg->inertial().pose().position().z());
  math::Quaternion q(_msg->inertial().pose().orientation().w(),
                     _msg->inertial().pose().orientation().x(),
                     _msg->inertial().pose().orientation().y(),
                     _msg->inertial().pose().orientation().z());

  this->Load(math::Pose(xyz, q));
}

/////////////////////////////////////////////////
void COMVisual::Load(const math::Pose &_pose)
{
  math::Vector3 p1(0, 0, -0.04);
  math::Vector3 p2(0, 0, 00.04);
  math::Vector3 p3(0, -0.04, 0);
  math::Vector3 p4(0, 00.04, 0);
  math::Vector3 p5(-0.04, 0, 0);
  math::Vector3 p6(00.04, 0, 0);
  p1 += _pose.pos;
  p2 += _pose.pos;
  p3 += _pose.pos;
  p4 += _pose.pos;
  p5 += _pose.pos;
  p6 += _pose.pos;
  p1 = _pose.rot.RotateVector(p1);
  p2 = _pose.rot.RotateVector(p2);
  p3 = _pose.rot.RotateVector(p3);
  p4 = _pose.rot.RotateVector(p4);
  p5 = _pose.rot.RotateVector(p5);
  p6 = _pose.rot.RotateVector(p6);

  this->crossLines = this->CreateDynamicLine(rendering::RENDERING_LINE_LIST);
  this->crossLines->setMaterial("Gazebo/Green");
  this->crossLines->AddPoint(p1);
  this->crossLines->AddPoint(p2);
  this->crossLines->AddPoint(p3);
  this->crossLines->AddPoint(p4);
  this->crossLines->AddPoint(p5);
  this->crossLines->AddPoint(p6);

  this->InsertMesh("unit_box");

  Ogre::MovableObject *boxObj =
    (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
          this->GetName()+"__BOX__", "unit_box"));
  boxObj->setVisibilityFlags(GZ_VISIBILITY_GUI);
  ((Ogre::Entity*)boxObj)->setMaterialName("__GAZEBO_TRANS_PURPLE_MATERIAL__");

  this->boxNode =
    this->sceneNode->createChildSceneNode(this->GetName() + "_BOX");

  this->boxNode->attachObject(boxObj);
  this->boxNode->setScale(0.02, 0.02, 0.02);
  this->boxNode->setPosition(_pose.pos.x, _pose.pos.y, _pose.pos.z);
  this->boxNode->setOrientation(Ogre::Quaternion(_pose.rot.w, _pose.rot.x,
                                                 _pose.rot.y, _pose.rot.z));

  this->SetVisibilityFlags(GZ_VISIBILITY_GUI);
}
