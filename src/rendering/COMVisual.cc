/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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

#include "common/MeshManager.hh"
#include "math/Vector3.hh"
#include "math/Quaternion.hh"
#include "math/Pose.hh"

#include "rendering/DynamicLines.hh"
#include "rendering/ogre.h"
#include "rendering/Scene.hh"
#include "rendering/COMVisual.hh"

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
void COMVisual::Load(ConstLinkPtr &_msg)
{
  Visual::Load();

  math::Vector3 xyz(_msg->inertial().pose().position().x(),
                    _msg->inertial().pose().position().y(),
                    _msg->inertial().pose().position().z());
  math::Quaternion q( _msg->inertial().pose().orientation().w(),
                      _msg->inertial().pose().orientation().x(),
                      _msg->inertial().pose().orientation().y(),
                      _msg->inertial().pose().orientation().z() );
  math::Vector3 p1(0, 0, -0.04);
  math::Vector3 p2(0, 0,  0.04);
  math::Vector3 p3(0, -0.04, 0);
  math::Vector3 p4(0,  0.04, 0);
  math::Vector3 p5(-0.04, 0, 0);
  math::Vector3 p6( 0.04, 0, 0);
  p1 += xyz;
  p2 += xyz;
  p3 += xyz;
  p4 += xyz;
  p5 += xyz;
  p6 += xyz;
  p1 = q.RotateVector(p1);
  p2 = q.RotateVector(p2);
  p3 = q.RotateVector(p3);
  p4 = q.RotateVector(p4);
  p5 = q.RotateVector(p5);
  p6 = q.RotateVector(p6);

  this->crossLines = this->CreateDynamicLine(rendering::RENDERING_LINE_LIST);
  this->crossLines->setMaterial("Gazebo/Green");
  this->crossLines->AddPoint(p1);
  this->crossLines->AddPoint(p2);
  this->crossLines->AddPoint(p3);
  this->crossLines->AddPoint(p4);
  this->crossLines->AddPoint(p5);
  this->crossLines->AddPoint(p6);

  Ogre::MovableObject *boxObj =
    (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
          this->GetName()+"__BOX__", "unit_box"));
  boxObj->setVisibilityFlags(GZ_VISIBILITY_GUI);
  ((Ogre::Entity*)boxObj)->setMaterialName("__GAZEBO_TRANS_PURPLE_MATERIAL__");

  this->boxNode =
    this->sceneNode->createChildSceneNode(this->GetName() + "_BOX");

  this->boxNode->attachObject(boxObj);
  this->boxNode->setScale(0.02, 0.02, 0.02);
  this->boxNode->setPosition(xyz.x,xyz.y,xyz.z);
  this->boxNode->setOrientation(Ogre::Quaternion(q.w, q.x, q.y, q.z));

  this->SetVisibilityFlags(GZ_VISIBILITY_GUI);
}
